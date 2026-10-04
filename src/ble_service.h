#pragma once

#include <Arduino.h>
#include <FS.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "navigation_fix.h"
#include "runtime_status.h"

namespace gps_logger {

class TrackLogger;

class BleService final : public NimBLEServerCallbacks,
                         public NimBLECharacteristicCallbacks {
 public:
  bool begin();
  void poll(const RuntimeStatus& status, TrackLogger& logger, std::uint32_t now_ms);
  void publishFix(const NavigationFix& fix);

  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] bool connected() const { return connected_.load(); }
  [[nodiscard]] bool secured() const { return secured_.load(); }
  [[nodiscard]] std::uint32_t passkey() const { return passkey_; }

 private:
  struct Command {
    char text[96] = {};
  };

  void onConnect(NimBLEServer* server, NimBLEConnInfo& connection) override;
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connection, int reason) override;
  std::uint32_t onPassKeyDisplay() override;
  void onAuthenticationComplete(NimBLEConnInfo& connection) override;
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connection) override;
  void onSubscribe(NimBLECharacteristic* characteristic, NimBLEConnInfo& connection,
                   std::uint16_t subscription) override;

  void enqueueCommands(const std::string& value);
  void processCommand(const Command& command, const RuntimeStatus& status,
                      TrackLogger& logger);
  void beginListing(TrackLogger& logger);
  void pollListing(const TrackLogger& logger, std::uint32_t now_ms);
  void beginDownload(std::uint32_t session_id, TrackLogger& logger);
  void pollDownload(std::uint32_t now_ms);
  void cancelOperations();
  bool notifyText(const String& text);
  bool notifyBytes(const std::uint8_t* data, std::size_t length);
  [[nodiscard]] std::size_t notificationPayloadSize() const;

  QueueHandle_t command_queue_ = nullptr;
  NimBLEServer* server_ = nullptr;
  NimBLECharacteristic* receive_characteristic_ = nullptr;
  NimBLECharacteristic* transmit_characteristic_ = nullptr;
  File listing_directory_;
  File download_file_;
  String listing_pending_;
  std::array<std::uint8_t, 244> download_buffer_{};
  std::atomic<bool> connected_{false};
  std::atomic<bool> secured_{false};
  std::atomic<bool> subscribed_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<std::uint32_t> dropped_commands_{0};
  std::atomic<std::uint16_t> connection_handle_{BLE_HS_CONN_HANDLE_NONE};
  std::uint32_t passkey_ = 0;
  std::uint32_t download_session_id_ = 0;
  std::uint32_t last_download_packet_ms_ = 0;
  std::uint32_t last_listing_packet_ms_ = 0;
  std::size_t download_remaining_ = 0;
  bool listing_ = false;
  bool listing_finished_ = false;
  bool live_enabled_ = false;
  bool ready_ = false;
};

}  // namespace gps_logger
