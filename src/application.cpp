#include "application.h"

#include <Arduino.h>

#include "hardware_config.h"
#include "navigation_fix.h"

namespace gps_logger {

void Application::begin() {
  Serial.begin(hardware::kDebugBaud);
  delay(200);
  Serial.printf("ESP32 GPS logger %s\n", GPS_LOGGER_VERSION);

  status_.display_ready = display_.begin();
  status_.sd_ready = logger_.begin();
  status_.ble_ready = ble_.begin();
  status_.gnss_ready = gnss_.begin();
  updateStatus();
}

void Application::tick() {
  const auto now = millis();
  retryHardware(now);

  if (gnss_.ready()) {
    gnss_.poll(logger_, now);
    NavigationFix new_fix;
    if (gnss_.takeLatestFix(new_fix)) {
      status_.fix = new_fix;
      status_.has_fix = hasUsablePosition(new_fix);
      printFix(new_fix);
      ble_.publishFix(new_fix);
    }
  }

  logger_.tick(now);
  updateStatus();
  ble_.poll(status_, logger_, now);
  display_.render(status_, now);
  delay(2);
}

void Application::retryHardware(std::uint32_t now_ms) {
  if (!logger_.ready() && now_ms - last_sd_attempt_ms_ >= hardware::kHardwareRetryIntervalMs) {
    last_sd_attempt_ms_ = now_ms;
    logger_.begin();
  }

  if (!gnss_.ready() &&
      now_ms - last_gnss_attempt_ms_ >= hardware::kHardwareRetryIntervalMs) {
    last_gnss_attempt_ms_ = now_ms;
    gnss_.begin();
  }
}

void Application::updateStatus() {
  status_.sd_ready = logger_.ready();
  status_.gnss_ready = gnss_.ready();
  status_.ble_ready = ble_.ready();
  status_.ble_connected = ble_.connected();
  status_.ble_secured = ble_.secured();
  status_.ble_passkey = ble_.passkey();
  status_.logging = status_.sd_ready && status_.gnss_ready;
  status_.session_id = logger_.sessionId();
  status_.bytes_written = logger_.bytesWritten();
  status_.dropped_bytes = logger_.droppedBytes();
  status_.write_failures = logger_.writeFailures();
  status_.gnss_buffer_high_water = gnss_.bufferHighWater();
}

void Application::printFix(const NavigationFix& fix) const {
  Serial.printf("%s fix=%s sats=%u lat=%s lon=%s hAcc=%.2fm%s\n",
                hasValidUtc(fix) ? formatUtc(fix.utc).c_str() : "time-unresolved",
                fixTypeLabel(fix.fix_type), fix.satellites,
                formatCoordinateE7(fix.latitude_e7).c_str(),
                formatCoordinateE7(fix.longitude_e7).c_str(),
                fix.horizontal_accuracy_mm / 1000.0,
                hasUsablePosition(fix) ? "" : " INVALID");
}

}  // namespace gps_logger
