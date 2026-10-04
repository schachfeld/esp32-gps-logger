#include "track_logger.h"

#include <Preferences.h>
#include <SD.h>
#include <SPI.h>

#include "hardware_config.h"

namespace gps_logger {

namespace {
SPIClass& sdSpi() {
  static SPIClass bus(VSPI);
  return bus;
}
}  // namespace

bool TrackLogger::begin() {
  if (ready_) {
    return true;
  }

  if (!mounted_) {
    sdSpi().begin(hardware::kSdClockPin, hardware::kSdMisoPin, hardware::kSdMosiPin,
                  hardware::kSdChipSelectPin);
    mounted_ = SD.begin(hardware::kSdChipSelectPin, sdSpi(), hardware::kSdClockHz);
    if (!mounted_ || SD.cardType() == CARD_NONE) {
      SD.end();
      mounted_ = false;
      return false;
    }
  }

  if (!SD.exists(hardware::kTrackDirectory) && !SD.mkdir(hardware::kTrackDirectory)) {
    SD.end();
    mounted_ = false;
    return false;
  }

  session_id_ = allocateSessionId();
  if (session_id_ == 0) {
    return false;
  }

  active_path_ = sessionPath(session_id_);
  file_ = SD.open(active_path_, FILE_APPEND);
  if (!file_ || file_.isDirectory()) {
    file_.close();
    SD.end();
    mounted_ = false;
    return false;
  }

  bytes_written_ = file_.size();
  last_flush_ms_ = millis();
  ready_ = true;
  Serial.printf("Logging raw UBX to %s\n", active_path_.c_str());
  return true;
}

void TrackLogger::tick(std::uint32_t now_ms) {
  if (ready_ && now_ms - last_flush_ms_ >= hardware::kFileFlushIntervalMs) {
    sync();
    last_flush_ms_ = now_ms;
  }
}

bool TrackLogger::append(const std::uint8_t* data, std::size_t length) {
  if (!ready_ || data == nullptr || length == 0) {
    noteDropped(length);
    return false;
  }

  const auto written = file_.write(data, length);
  bytes_written_ += written;
  if (written != length) {
    noteDropped(length - written);
    failWrite();
    return false;
  }
  return true;
}

void TrackLogger::noteDropped(std::size_t length) { dropped_bytes_ += length; }

void TrackLogger::sync() {
  if (ready_) {
    file_.flush();
  }
}

void TrackLogger::close() {
  if (file_) {
    file_.flush();
    file_.close();
  }
  ready_ = false;
}

File TrackLogger::openSessionForRead(std::uint32_t session_id) {
  if (!mounted_ || session_id == 0 || session_id == session_id_) {
    return {};
  }
  return SD.open(sessionPath(session_id), FILE_READ);
}

String TrackLogger::sessionPath(std::uint32_t session_id) {
  char path[32] = {};
  std::snprintf(path, sizeof(path), "%s/S%08lu.ubx", hardware::kTrackDirectory,
                static_cast<unsigned long>(session_id));
  return path;
}

std::uint32_t TrackLogger::allocateSessionId() {
  Preferences preferences;
  if (!preferences.begin("gps-logger", false)) {
    return 0;
  }

  auto candidate = preferences.getUInt("next-session", 1);
  if (candidate == 0) {
    candidate = 1;
  }

  constexpr std::uint32_t kMaximumCollisionChecks = 10000;
  std::uint32_t checks = 0;
  while (SD.exists(sessionPath(candidate)) && checks++ < kMaximumCollisionChecks) {
    ++candidate;
    if (candidate == 0) {
      candidate = 1;
    }
  }

  if (checks >= kMaximumCollisionChecks) {
    preferences.end();
    return 0;
  }

  auto next = candidate + 1;
  if (next == 0) {
    next = 1;
  }
  const auto stored = preferences.putUInt("next-session", next);
  preferences.end();
  return stored == sizeof(std::uint32_t) ? candidate : 0;
}

void TrackLogger::failWrite() {
  ++write_failures_;
  Serial.println("SD write failed; closing the active session");
  close();
  SD.end();
  mounted_ = false;
}

}  // namespace gps_logger
