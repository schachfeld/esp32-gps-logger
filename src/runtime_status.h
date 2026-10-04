#pragma once

#include <cstddef>
#include <cstdint>

#include "navigation_fix.h"

namespace gps_logger {

struct RuntimeStatus {
  bool display_ready = false;
  bool sd_ready = false;
  bool gnss_ready = false;
  bool has_fix = false;
  bool logging = false;
  bool ble_ready = false;
  bool ble_connected = false;
  bool ble_secured = false;
  NavigationFix fix;
  std::uint32_t ble_passkey = 0;
  std::uint32_t session_id = 0;
  std::size_t bytes_written = 0;
  std::size_t dropped_bytes = 0;
  std::uint32_t write_failures = 0;
  std::uint16_t gnss_buffer_high_water = 0;
};

}  // namespace gps_logger
