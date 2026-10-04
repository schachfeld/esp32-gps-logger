#include "status_display.h"

#include <Wire.h>

#include "hardware_config.h"
#include "navigation_fix.h"

namespace gps_logger {

StatusDisplay::StatusDisplay()
    : display_(hardware::kOledWidth, hardware::kOledHeight, &Wire, -1) {}

bool StatusDisplay::begin() {
  Wire.begin(hardware::kI2cSdaPin, hardware::kI2cSclPin);
  ready_ = display_.begin(SSD1306_SWITCHCAPVCC, hardware::kOledAddress, true, false);
  if (!ready_) {
    Serial.println("OLED not detected; continuing headless");
    return false;
  }
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println("GPS logger starting");
  display_.display();
  return true;
}

void StatusDisplay::render(const RuntimeStatus& status, std::uint32_t now_ms) {
  if (!ready_ || now_ms - last_render_ms_ < 500) {
    return;
  }
  last_render_ms_ = now_ms;

  display_.clearDisplay();
  display_.setCursor(0, 0);

  if (!status.sd_ready) {
    display_.println("SD: not ready");
  } else {
    display_.printf("LOG S%08lu\n", static_cast<unsigned long>(status.session_id));
  }

  if (!status.gnss_ready) {
    display_.println("GNSS: reconnecting");
  } else {
    display_.printf("%s %usv PDOP %.2f\n", fixTypeLabel(status.fix.fix_type),
                    status.fix.satellites, status.fix.position_dop_centi / 100.0);
  }

  if (status.has_fix) {
    display_.printf("%.5f %.5f\n", status.fix.latitude_e7 / 10000000.0,
                    status.fix.longitude_e7 / 10000000.0);
  } else {
    display_.println("Waiting for valid fix");
  }

  if (status.write_failures > 0) {
    display_.printf("SD errors: %lu", static_cast<unsigned long>(status.write_failures));
  } else if (status.ble_connected && status.ble_secured) {
    display_.print("BLE connected");
  } else if (status.ble_ready) {
    display_.printf("BLE PIN %06lu", static_cast<unsigned long>(status.ble_passkey));
  } else {
    display_.printf("UBX %lu bytes", static_cast<unsigned long>(status.bytes_written));
  }
  display_.display();
}

}  // namespace gps_logger
