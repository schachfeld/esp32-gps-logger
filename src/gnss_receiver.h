#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <array>
#include <cstdint>

#include "navigation_fix.h"

namespace gps_logger {

class TrackLogger;

class GnssReceiver {
 public:
  bool begin();
  void poll(TrackLogger& logger, std::uint32_t now_ms);
  bool takeLatestFix(NavigationFix& destination);

  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] std::uint16_t bufferHighWater() const { return buffer_high_water_; }

 private:
  static void onPvt(UBX_NAV_PVT_data_t* pvt);
  void capturePvt(const UBX_NAV_PVT_data_t& pvt);
  bool connectAtBaud(std::uint32_t baud);
  bool configure();
  void drainFileBuffer(TrackLogger& logger);

  static GnssReceiver* active_instance_;

  SFE_UBLOX_GNSS gnss_;
  std::array<std::uint8_t, 512> drain_buffer_{};
  NavigationFix latest_fix_{};
  std::uint32_t last_message_ms_ = 0;
  std::uint16_t buffer_high_water_ = 0;
  bool buffer_configured_ = false;
  bool has_new_fix_ = false;
  bool ready_ = false;
};

}  // namespace gps_logger
