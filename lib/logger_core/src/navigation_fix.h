#pragma once

#include <cstdint>
#include <string>

namespace gps_logger {

struct UtcDateTime {
  std::uint16_t year = 0;
  std::uint8_t month = 0;
  std::uint8_t day = 0;
  std::uint8_t hour = 0;
  std::uint8_t minute = 0;
  std::uint8_t second = 0;
  std::uint16_t millisecond = 0;
  bool valid_date = false;
  bool valid_time = false;
  bool fully_resolved = false;
};

struct NavigationFix {
  UtcDateTime utc;
  std::int32_t latitude_e7 = 0;
  std::int32_t longitude_e7 = 0;
  std::int32_t altitude_msl_mm = 0;
  std::uint32_t horizontal_accuracy_mm = 0;
  std::uint32_t vertical_accuracy_mm = 0;
  std::int32_t ground_speed_mm_s = 0;
  std::int32_t heading_e5 = 0;
  std::uint16_t position_dop_centi = 0;
  std::uint8_t satellites = 0;
  std::uint8_t fix_type = 0;
  bool gnss_fix_ok = false;
  bool invalid_llh = true;
};

bool hasValidUtc(const NavigationFix& fix);
bool hasUsablePosition(const NavigationFix& fix);
const char* fixTypeLabel(std::uint8_t fix_type);
std::string formatCoordinateE7(std::int32_t coordinate_e7);
std::string formatUtc(const UtcDateTime& utc);

}  // namespace gps_logger
