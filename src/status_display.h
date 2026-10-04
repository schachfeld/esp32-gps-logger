#pragma once

#include <Adafruit_SSD1306.h>

#include <cstdint>

#include "runtime_status.h"

namespace gps_logger {

class StatusDisplay {
 public:
  StatusDisplay();
  bool begin();
  void render(const RuntimeStatus& status, std::uint32_t now_ms);

  [[nodiscard]] bool ready() const { return ready_; }

 private:
  Adafruit_SSD1306 display_;
  std::uint32_t last_render_ms_ = 0;
  bool ready_ = false;
};

}  // namespace gps_logger
