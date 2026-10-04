#pragma once

#include <cstddef>
#include <cstdint>

namespace gps_logger::hardware {

// Existing ESP32 wiring. Change these only if the physical wiring changes.
constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;
constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;
constexpr int kSdChipSelectPin = 5;
constexpr int kSdClockPin = 18;
constexpr int kSdMisoPin = 19;
constexpr int kSdMosiPin = 23;

constexpr std::uint8_t kOledAddress = 0x3C;
constexpr int kOledWidth = 128;
constexpr int kOledHeight = 32;

constexpr std::uint32_t kDebugBaud = 115200;
constexpr std::uint32_t kGnssBaud = 38400;
constexpr std::uint32_t kGnssFallbackBaud = 9600;
constexpr std::uint8_t kNavigationFrequencyHz = 1;
constexpr std::size_t kGnssUartBufferBytes = 4096;
constexpr std::uint16_t kGnssFileBufferBytes = 8192;
constexpr std::uint32_t kGnssSilenceTimeoutMs = 10000;

constexpr std::uint32_t kSdClockHz = 10000000;
constexpr std::uint32_t kFileFlushIntervalMs = 5000;
constexpr std::uint32_t kHardwareRetryIntervalMs = 10000;
constexpr char kTrackDirectory[] = "/tracks";

}  // namespace gps_logger::hardware
