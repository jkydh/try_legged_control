#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <termios.h>
#include <vector>

namespace legged {

struct H30Sample {
  uint16_t tid = 0;
  std::array<double, 4> orientation{{0., 0., 0., 1.}};  // x, y, z, w
  std::array<double, 3> angularVelocity{};              // rad/s, sensor axes
  std::array<double, 3> linearAcceleration{};           // m/s^2, includes gravity
};

// Receive-only YESENSE parser. A sample requires acc, gyro and quaternion in
// the SAME checked frame; fields from old frames are never merged.
class H30Parser {
 public:
  std::vector<H30Sample> feed(const uint8_t* data, size_t size);
  size_t validFrames() const { return validFrames_; }
  size_t checksumErrors() const { return checksumErrors_; }
  size_t payloadErrors() const { return payloadErrors_; }

 private:
  bool decode(const uint8_t* payload, size_t size, H30Sample& sample) const;
  std::vector<uint8_t> buffer_;
  size_t validFrames_ = 0, checksumErrors_ = 0, payloadErrors_ = 0;
};

// Nonblocking Linux serial port. No UART payload is ever transmitted.
class H30Serial {
 public:
  H30Serial(const std::string& port, int baud);
  ~H30Serial();
  H30Serial(const H30Serial&) = delete;
  H30Serial& operator=(const H30Serial&) = delete;
  std::vector<H30Sample> poll();
  const H30Parser& parser() const { return parser_; }

 private:
  void closePort();
  int fd_ = -1;
  termios saved_{};
  bool savedValid_ = false;
  H30Parser parser_;
};

}  // namespace legged
