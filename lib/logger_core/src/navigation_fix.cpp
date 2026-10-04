#include "navigation_fix.h"

#include <cstdio>

namespace gps_logger {

bool hasValidUtc(const NavigationFix& fix) {
  return fix.utc.valid_date && fix.utc.valid_time && fix.utc.fully_resolved &&
         fix.utc.year >= 2000 && fix.utc.month >= 1 && fix.utc.month <= 12 && fix.utc.day >= 1 &&
         fix.utc.day <= 31 && fix.utc.hour <= 23 && fix.utc.minute <= 59 && fix.utc.second <= 60 &&
         fix.utc.millisecond <= 999;
}

bool hasUsablePosition(const NavigationFix& fix) {
  const bool position_fix = fix.fix_type == 2 || fix.fix_type == 3 || fix.fix_type == 4;
  return position_fix && fix.gnss_fix_ok && !fix.invalid_llh;
}

const char* fixTypeLabel(std::uint8_t fix_type) {
  switch (fix_type) {
    case 1:
      return "DR";
    case 2:
      return "2D";
    case 3:
      return "3D";
    case 4:
      return "3D+DR";
    case 5:
      return "TIME";
    default:
      return "NONE";
  }
}

std::string formatCoordinateE7(std::int32_t coordinate_e7) {
  const bool negative = coordinate_e7 < 0;
  const auto magnitude = static_cast<std::uint32_t>(
      negative ? -static_cast<std::int64_t>(coordinate_e7) : coordinate_e7);
  const auto degrees = magnitude / 10000000U;
  const auto fraction = magnitude % 10000000U;

  char output[20] = {};
  std::snprintf(output, sizeof(output), "%s%u.%07u", negative ? "-" : "", degrees, fraction);
  return output;
}

std::string formatUtc(const UtcDateTime& utc) {
  char output[32] = {};
  std::snprintf(output, sizeof(output), "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ", utc.year,
                utc.month, utc.day, utc.hour, utc.minute, utc.second, utc.millisecond);
  return output;
}

}  // namespace gps_logger

