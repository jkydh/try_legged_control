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
  std::array<double, 3> rpy{};                          // rad, device Euler output
  double temperature = 0.;                            // degrees C
  bool hasOrientation = false, hasAngularVelocity = false, hasLinearAcceleration = false;
  bool hasRpy = false, hasTemperature = false;
  bool complete() const { return hasOrientation && hasAngularVelocity && hasLinearAcceleration; }
  bool hasImuData() const { return hasOrientation || hasAngularVelocity || hasLinearAcceleration || hasRpy; }
};

// Receive-only YESENSE parser. A sample requires acc, gyro and quaternion in
// the SAME checked frame by default; diagnostic tools can explicitly accept
// partial frames. Presence flags always refer to this frame, never old data.
class H30Parser {
 public:
  explicit H30Parser(bool requireComplete = true) : requireComplete_(requireComplete) {}
  std::vector<H30Sample> feed(const uint8_t* data, size_t size);
  size_t validFrames() const { return validFrames_; }
  size_t checksumErrors() const { return checksumErrors_; }
  size_t payloadErrors() const { return payloadErrors_; }

 private:
  bool decode(const uint8_t* payload, size_t size, H30Sample& sample) const;
  bool requireComplete_;
  std::vector<uint8_t> buffer_;
  size_t validFrames_ = 0, checksumErrors_ = 0, payloadErrors_ = 0;
};

// Nonblocking Linux serial port. No UART payload is ever transmitted.
class H30Serial {
 public:
  H30Serial(const std::string& port, int baud, bool requireComplete = true);
  ~H30Serial();
  H30Serial(const H30Serial&) = delete;
  H30Serial& operator=(const H30Serial&) = delete;
  std::vector<H30Sample> poll();
  const H30Parser& parser() const { return parser_; }
  size_t bytesReceived() const { return bytesReceived_; }

 private:
  void closePort();
  int fd_ = -1;
  termios saved_{};
  bool savedValid_ = false;
  H30Parser parser_;
  size_t bytesReceived_ = 0;
};

}  // namespace legged
