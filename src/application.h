#pragma once

#include <cstdint>

#include "gnss_receiver.h"
#include "runtime_status.h"
#include "status_display.h"
#include "track_logger.h"

namespace gps_logger {

class Application {
 public:
  void begin();
  void tick();

 private:
  void retryHardware(std::uint32_t now_ms);
  void updateStatus();
  void printFix(const NavigationFix& fix) const;

  StatusDisplay display_;
  TrackLogger logger_;
  GnssReceiver gnss_;
  RuntimeStatus status_;
  std::uint32_t last_sd_attempt_ms_ = 0;
  std::uint32_t last_gnss_attempt_ms_ = 0;
};

}  // namespace gps_logger
