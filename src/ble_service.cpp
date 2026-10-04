#include "ble_service.h"

#include <esp_system.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "navigation_fix.h"
#include "track_logger.h"

namespace gps_logger {

namespace {

constexpr char kServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kReceiveUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kTransmitUuid[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
constexpr std::uint32_t kDownloadPacketIntervalMs = 8;
constexpr std::uint32_t kListingPacketIntervalMs = 20;

bool parseSessionName(const char* path, std::uint32_t& session_id) {
  if (path == nullptr) {
    return false;
  }

  const auto* name = std::strrchr(path, '/');
  name = name == nullptr ? path : name + 1;
  if (std::strlen(name) != 13 || name[0] != 'S' ||
      std::strcmp(name + 9, ".ubx") != 0) {
    return false;
  }

  std::uint32_t value = 0;
  for (std::size_t index = 1; index <= 8; ++index) {
    if (!std::isdigit(static_cast<unsigned char>(name[index]))) {
      return false;
    }
    value = value * 10U + static_cast<std::uint32_t>(name[index] - '0');
  }
  session_id = value;
  return value != 0;
}

bool parseRequestedSession(String value, std::uint32_t& session_id) {
  value.trim();
  value.toUpperCase();
  if (value.endsWith(".UBX")) {
    value.remove(value.length() - 4);
  }
  if (value.startsWith("S")) {
    value.remove(0, 1);
  }
  if (value.isEmpty() || value.length() > 10) {
    return false;
  }

  std::uint64_t result = 0;
  for (std::size_t index = 0; index < value.length(); ++index) {
    const auto character = value[index];
    if (character < '0' || character > '9') {
      return false;
    }
    result = result * 10U + static_cast<unsigned>(character - '0');
    if (result > std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
  }
  session_id = static_cast<std::uint32_t>(result);
  return session_id != 0;
}

}  // namespace

bool BleService::begin() {
  if (ready_) {
    return true;
  }

  command_queue_ = xQueueCreate(8, sizeof(Command));
  if (command_queue_ == nullptr) {
    Serial.println("BLE command queue allocation failed");
    return false;
  }

  char device_name[24] = {};
  const auto chip_id = static_cast<std::uint32_t>(ESP.getEfuseMac());
  std::snprintf(device_name, sizeof(device_name), "GPS-Logger-%06lX",
                static_cast<unsigned long>(chip_id & 0xFFFFFFU));
  passkey_ = 100000U + esp_random() % 900000U;

  if (!NimBLEDevice::init(device_name)) {
    Serial.println("BLE initialization failed");
    return false;
  }
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
  NimBLEDevice::setSecurityPasskey(passkey_);

  server_ = NimBLEDevice::createServer();
  if (server_ == nullptr) {
    return false;
  }
  server_->setCallbacks(this, false);
  server_->advertiseOnDisconnect(true);

  auto* service = server_->createService(kServiceUuid);
  if (service == nullptr) {
    return false;
  }
  receive_characteristic_ = service->createCharacteristic(
      kReceiveUuid,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE_ENC,
      sizeof(Command::text) - 1);
  transmit_characteristic_ = service->createCharacteristic(
      kTransmitUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC,
      download_buffer_.size());
  if (receive_characteristic_ == nullptr || transmit_characteristic_ == nullptr) {
    return false;
  }
  receive_characteristic_->setCallbacks(this);
  transmit_characteristic_->setCallbacks(this);

  if (!server_->start()) {
    Serial.println("BLE GATT server failed to start");
    return false;
  }
  auto* advertising = NimBLEDevice::getAdvertising();
  advertising->setName(device_name);
  advertising->addServiceUUID(kServiceUuid);
  advertising->enableScanResponse(true);
  if (!advertising->start()) {
    Serial.println("BLE advertising failed to start");
    return false;
  }

  ready_ = true;
  Serial.printf("BLE %s ready; pairing PIN %06lu\n", device_name,
                static_cast<unsigned long>(passkey_));
  return true;
}

void BleService::poll(const RuntimeStatus& status, TrackLogger& logger,
                      std::uint32_t now_ms) {
  if (!ready_) {
    return;
  }

  if (cancel_requested_.exchange(false)) {
    cancelOperations();
    xQueueReset(command_queue_);
    live_enabled_ = false;
  }

  const auto dropped = dropped_commands_.exchange(0);
  if (dropped > 0 && !download_file_) {
    notifyText("ERR COMMAND_QUEUE_FULL\n");
  }

  Command command;
  if (xQueueReceive(command_queue_, &command, 0) == pdTRUE) {
    processCommand(command, status, logger);
    return;
  }

  if (download_file_) {
    pollDownload(now_ms);
  } else if (listing_) {
    pollListing(logger, now_ms);
  }
}

void BleService::publishFix(const NavigationFix& fix) {
  if (!ready_ || !live_enabled_ || !connected() || !secured() || !subscribed_.load() ||
      listing_ || download_file_) {
    return;
  }

  const auto latitude = formatCoordinateE7(fix.latitude_e7);
  const auto longitude = formatCoordinateE7(fix.longitude_e7);
  const auto timestamp = hasValidUtc(fix) ? formatUtc(fix.utc) : "unresolved";
  char message[192] = {};
  std::snprintf(message, sizeof(message), "FIX %s,%s,%s,%.3f,%u,%s,%u\n",
                timestamp.c_str(), latitude.c_str(), longitude.c_str(),
                fix.altitude_msl_mm / 1000.0, fix.satellites,
                fixTypeLabel(fix.fix_type), fix.horizontal_accuracy_mm);
  notifyText(message);
}

void BleService::onConnect(NimBLEServer* server, NimBLEConnInfo& connection) {
  connection_handle_.store(connection.getConnHandle());
  connected_.store(true);
  secured_.store(connection.isEncrypted() && connection.isAuthenticated());
  subscribed_.store(false);
  server->updateConnParams(connection.getConnHandle(), 12, 24, 0, 400);
  Serial.println("BLE client connected");
}

void BleService::onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) {
  connected_.store(false);
  secured_.store(false);
  subscribed_.store(false);
  connection_handle_.store(BLE_HS_CONN_HANDLE_NONE);
  cancel_requested_.store(true);
  Serial.printf("BLE client disconnected (reason %d)\n", reason);
}

std::uint32_t BleService::onPassKeyDisplay() { return passkey_; }

void BleService::onAuthenticationComplete(NimBLEConnInfo& connection) {
  if (!connection.isEncrypted() || !connection.isAuthenticated()) {
    secured_.store(false);
    if (server_ != nullptr) {
      server_->disconnect(connection.getConnHandle());
    }
    Serial.println("BLE authentication failed");
    return;
  }
  secured_.store(true);
  Serial.println("BLE link authenticated and encrypted");
}

void BleService::onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) {
  if (characteristic == receive_characteristic_) {
    enqueueCommands(characteristic->getValue());
  }
}

void BleService::onSubscribe(NimBLECharacteristic* characteristic, NimBLEConnInfo&,
                             std::uint16_t subscription) {
  if (characteristic == transmit_characteristic_) {
    subscribed_.store((subscription & 1U) != 0);
  }
}

void BleService::enqueueCommands(const std::string& value) {
  std::size_t start = 0;
  while (start < value.size()) {
    const auto end = value.find_first_of("\r\n", start);
    const auto length = (end == std::string::npos ? value.size() : end) - start;
    if (length > 0) {
      if (length >= sizeof(Command::text)) {
        ++dropped_commands_;
      } else {
        Command command;
        std::memcpy(command.text, value.data() + start, length);
        command.text[length] = '\0';
        if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
          ++dropped_commands_;
        }
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
}

void BleService::processCommand(const Command& input, const RuntimeStatus& status,
                                TrackLogger& logger) {
  String command(input.text);
  command.trim();
  command.toUpperCase();

  if (download_file_) {
    if (command == "CANCEL") {
      cancelOperations();
      notifyText("CANCELLED\n");
    }
    return;
  }

  if (command == "HELP") {
    notifyText("OK COMMANDS HELP STATUS LIVE ON LIVE OFF LIST GET <session> CANCEL\n");
  } else if (command == "STATUS") {
    char response[192] = {};
    std::snprintf(response, sizeof(response),
                  "STATUS gnss=%u sd=%u fix=%s sats=%u session=S%08lu bytes=%lu dropped=%lu\n",
                  status.gnss_ready, status.sd_ready, fixTypeLabel(status.fix.fix_type),
                  status.fix.satellites, static_cast<unsigned long>(status.session_id),
                  static_cast<unsigned long>(status.bytes_written),
                  static_cast<unsigned long>(status.dropped_bytes));
    notifyText(response);
  } else if (command == "LIVE ON") {
    live_enabled_ = true;
    notifyText("OK LIVE ON\n");
  } else if (command == "LIVE OFF") {
    live_enabled_ = false;
    notifyText("OK LIVE OFF\n");
  } else if (command == "LIST") {
    beginListing(logger);
  } else if (command.startsWith("GET ")) {
    std::uint32_t session_id = 0;
    if (!parseRequestedSession(command.substring(4), session_id)) {
      notifyText("ERR INVALID_SESSION\n");
    } else {
      beginDownload(session_id, logger);
    }
  } else if (command == "CANCEL") {
    const bool was_busy = listing_;
    cancelOperations();
    notifyText(was_busy ? "CANCELLED\n" : "OK IDLE\n");
  } else if (!command.isEmpty()) {
    notifyText("ERR UNKNOWN_COMMAND\n");
  }
}

void BleService::beginListing(TrackLogger& logger) {
  cancelOperations();
  listing_directory_ = logger.openTrackDirectory();
  if (!listing_directory_ || !listing_directory_.isDirectory()) {
    listing_directory_.close();
    notifyText("ERR STORAGE_UNAVAILABLE\n");
    return;
  }
  listing_ = true;
  listing_finished_ = false;
  listing_pending_ = "BEGIN LIST\n";
}

void BleService::pollListing(const TrackLogger& logger, std::uint32_t now_ms) {
  if (now_ms - last_listing_packet_ms_ < kListingPacketIntervalMs) {
    return;
  }
  last_listing_packet_ms_ = now_ms;

  if (!listing_pending_.isEmpty()) {
    if (!notifyText(listing_pending_)) {
      return;
    }
    listing_pending_.clear();
    if (listing_finished_) {
      listing_ = false;
    }
    return;
  }

  File entry;
  while ((entry = listing_directory_.openNextFile())) {
    std::uint32_t session_id = 0;
    if (!entry.isDirectory() && parseSessionName(entry.name(), session_id)) {
      char line[80] = {};
      std::snprintf(line, sizeof(line), "FILE S%08lu %lu%s\n",
                    static_cast<unsigned long>(session_id),
                    static_cast<unsigned long>(entry.size()),
                    session_id == logger.sessionId() ? " ACTIVE" : "");
      entry.close();
      listing_pending_ = line;
      return;
    }
    entry.close();
  }

  listing_directory_.close();
  listing_finished_ = true;
  listing_pending_ = "END LIST\n";
}

void BleService::beginDownload(std::uint32_t session_id, TrackLogger& logger) {
  cancelOperations();
  if (session_id == logger.sessionId()) {
    notifyText("ERR ACTIVE_SESSION\n");
    return;
  }

  download_file_ = logger.openSessionForRead(session_id);
  if (!download_file_ || download_file_.isDirectory()) {
    download_file_.close();
    notifyText("ERR SESSION_NOT_FOUND\n");
    return;
  }

  download_session_id_ = session_id;
  download_remaining_ = download_file_.size();
  char header[64] = {};
  std::snprintf(header, sizeof(header), "BEGIN S%08lu %lu\n",
                static_cast<unsigned long>(session_id),
                static_cast<unsigned long>(download_remaining_));
  if (!notifyText(header)) {
    cancelOperations();
  }
}

void BleService::pollDownload(std::uint32_t now_ms) {
  if (now_ms - last_download_packet_ms_ < kDownloadPacketIntervalMs) {
    return;
  }
  last_download_packet_ms_ = now_ms;

  if (download_remaining_ == 0) {
    const auto completed_session = download_session_id_;
    cancelOperations();
    char footer[32] = {};
    std::snprintf(footer, sizeof(footer), "END S%08lu\n",
                  static_cast<unsigned long>(completed_session));
    notifyText(footer);
    return;
  }

  const auto payload_size = notificationPayloadSize();
  const auto requested = std::min({download_remaining_, payload_size, download_buffer_.size()});
  const auto previous_position = download_file_.position();
  const auto bytes_read = download_file_.read(download_buffer_.data(), requested);
  if (bytes_read == 0) {
    cancelOperations();
    notifyText("ERR STORAGE_READ\n");
    return;
  }
  if (!notifyBytes(download_buffer_.data(), bytes_read)) {
    download_file_.seek(previous_position);
    return;
  }
  download_remaining_ -= bytes_read;
}

void BleService::cancelOperations() {
  if (listing_directory_) {
    listing_directory_.close();
  }
  if (download_file_) {
    download_file_.close();
  }
  listing_ = false;
  listing_finished_ = false;
  listing_pending_.clear();
  download_session_id_ = 0;
  download_remaining_ = 0;
}

bool BleService::notifyText(const String& text) {
  const auto* data = reinterpret_cast<const std::uint8_t*>(text.c_str());
  std::size_t offset = 0;
  while (offset < text.length()) {
    const auto length = std::min(notificationPayloadSize(), text.length() - offset);
    if (!notifyBytes(data + offset, length)) {
      return false;
    }
    offset += length;
  }
  return true;
}

bool BleService::notifyBytes(const std::uint8_t* data, std::size_t length) {
  if (!connected() || !secured() || !subscribed_.load() || transmit_characteristic_ == nullptr ||
      data == nullptr || length == 0) {
    return false;
  }
  return transmit_characteristic_->notify(data, length, connection_handle_.load());
}

std::size_t BleService::notificationPayloadSize() const {
  if (server_ == nullptr || !connected()) {
    return 20;
  }
  const auto mtu = server_->getPeerMTU(connection_handle_.load());
  if (mtu <= 3) {
    return 20;
  }
  return std::min<std::size_t>(mtu - 3U, download_buffer_.size());
}

}  // namespace gps_logger
