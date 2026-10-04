#pragma once

#include <Arduino.h>
#include <FS.h>

#include <cstddef>
#include <cstdint>

namespace gps_logger {

class TrackLogger {
 public:
  bool begin();
  void tick(std::uint32_t now_ms);
  bool append(const std::uint8_t* data, std::size_t length);
  void noteDropped(std::size_t length);
  void sync();
  void close();

  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] bool mounted() const { return mounted_; }
  [[nodiscard]] std::uint32_t sessionId() const { return session_id_; }
  [[nodiscard]] std::size_t bytesWritten() const { return bytes_written_; }
  [[nodiscard]] std::size_t droppedBytes() const { return dropped_bytes_; }
  [[nodiscard]] std::uint32_t writeFailures() const { return write_failures_; }
  [[nodiscard]] const String& activePath() const { return active_path_; }

  File openSessionForRead(std::uint32_t session_id);
  static String sessionPath(std::uint32_t session_id);

 private:
  std::uint32_t allocateSessionId();
  void failWrite();

  File file_;
  String active_path_;
  std::uint32_t session_id_ = 0;
  std::size_t bytes_written_ = 0;
  std::size_t dropped_bytes_ = 0;
  std::uint32_t write_failures_ = 0;
  std::uint32_t last_flush_ms_ = 0;
  bool mounted_ = false;
  bool ready_ = false;
};

}  // namespace gps_logger
