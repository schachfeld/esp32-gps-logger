#include "gnss_receiver.h"

#include <algorithm>

#include "hardware_config.h"
#include "track_logger.h"

namespace gps_logger {

GnssReceiver* GnssReceiver::active_instance_ = nullptr;

bool GnssReceiver::begin() {
  if (ready_) {
    return true;
  }

  if (!buffer_configured_) {
    gnss_.setFileBufferSize(hardware::kGnssFileBufferBytes);
    Serial2.setRxBufferSize(hardware::kGnssUartBufferBytes);
    buffer_configured_ = true;
  }

  active_instance_ = this;
  if (!connectAtBaud(hardware::kGnssBaud) && !connectAtBaud(hardware::kGnssFallbackBaud)) {
    Serial.println("GNSS not detected at 38400 or 9600 baud");
    return false;
  }

  if (!configure()) {
    Serial.println("GNSS detected but configuration failed");
    return false;
  }

  last_message_ms_ = millis();
  ready_ = true;
  Serial.println("GNSS configured for 1 Hz UBX-NAV-PVT");
  return true;
}

void GnssReceiver::poll(TrackLogger& logger, std::uint32_t now_ms) {
  if (!ready_) {
    return;
  }

  gnss_.checkUblox();
  gnss_.checkCallbacks();
  drainFileBuffer(logger);

  buffer_high_water_ = std::max(buffer_high_water_, gnss_.getMaxFileBufferAvail());
  if (now_ms - last_message_ms_ > hardware::kGnssSilenceTimeoutMs) {
    Serial.println("GNSS timed out; scheduling reconnect");
    ready_ = false;
  }
}

bool GnssReceiver::takeLatestFix(NavigationFix& destination) {
  if (!has_new_fix_) {
    return false;
  }
  destination = latest_fix_;
  has_new_fix_ = false;
  return true;
}

void GnssReceiver::onPvt(UBX_NAV_PVT_data_t* pvt) {
  if (active_instance_ != nullptr && pvt != nullptr) {
    active_instance_->capturePvt(*pvt);
  }
}

void GnssReceiver::capturePvt(const UBX_NAV_PVT_data_t& pvt) {
  latest_fix_.utc = {pvt.year,
                     pvt.month,
                     pvt.day,
                     pvt.hour,
                     pvt.min,
                     pvt.sec,
                     static_cast<std::uint16_t>(pvt.iTOW % 1000U),
                     pvt.valid.bits.validDate == 1,
                     pvt.valid.bits.validTime == 1,
                     pvt.valid.bits.fullyResolved == 1};
  latest_fix_.latitude_e7 = pvt.lat;
  latest_fix_.longitude_e7 = pvt.lon;
  latest_fix_.altitude_msl_mm = pvt.hMSL;
  latest_fix_.horizontal_accuracy_mm = pvt.hAcc;
  latest_fix_.vertical_accuracy_mm = pvt.vAcc;
  latest_fix_.ground_speed_mm_s = pvt.gSpeed;
  latest_fix_.heading_e5 = pvt.headMot;
  latest_fix_.position_dop_centi = pvt.pDOP;
  latest_fix_.satellites = pvt.numSV;
  latest_fix_.fix_type = pvt.fixType;
  latest_fix_.gnss_fix_ok = pvt.flags.bits.gnssFixOK == 1;
  latest_fix_.invalid_llh = pvt.flags3.bits.invalidLlh == 1;
  has_new_fix_ = true;
  last_message_ms_ = millis();
}

bool GnssReceiver::connectAtBaud(std::uint32_t baud) {
  Serial.printf("Trying GNSS at %lu baud\n", static_cast<unsigned long>(baud));
  Serial2.end();
  Serial2.begin(baud, SERIAL_8N1, hardware::kGnssRxPin, hardware::kGnssTxPin);
  delay(100);
  if (!gnss_.begin(Serial2)) {
    return false;
  }

  if (baud != hardware::kGnssBaud) {
    gnss_.setSerialRate(hardware::kGnssBaud);
    delay(100);
    Serial2.updateBaudRate(hardware::kGnssBaud);
    delay(100);
    if (!gnss_.isConnected()) {
      return false;
    }
  }
  return true;
}

bool GnssReceiver::configure() {
  const bool output_configured = gnss_.setUART1Output(COM_TYPE_UBX);
  const bool rate_configured = gnss_.setNavigationFrequency(hardware::kNavigationFrequencyHz);
  const bool callback_configured = gnss_.setAutoPVTcallbackPtr(&GnssReceiver::onPvt);
  if (!(output_configured && rate_configured && callback_configured)) {
    return false;
  }
  gnss_.logNAVPVT();
  return true;
}

void GnssReceiver::drainFileBuffer(TrackLogger& logger) {
  while (gnss_.fileBufferAvailable() > 0) {
    const auto available = gnss_.fileBufferAvailable();
    const auto requested = static_cast<std::uint16_t>(
        std::min<std::size_t>(available, drain_buffer_.size()));
    const auto extracted = gnss_.extractFileBufferData(drain_buffer_.data(), requested);
    if (extracted == 0) {
      return;
    }
    if (!logger.ready()) {
      logger.noteDropped(extracted);
      continue;
    }
    logger.append(drain_buffer_.data(), extracted);
  }
}

}  // namespace gps_logger
