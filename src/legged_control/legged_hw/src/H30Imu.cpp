#include "legged_hw/H30Imu.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <iterator>
#include <stdexcept>
#include <system_error>
#include <sys/ioctl.h>
#include <unistd.h>

namespace legged {
namespace {
double scaledInt32(const uint8_t* data) {
  const uint32_t value = uint32_t(data[0]) | (uint32_t(data[1]) << 8) |
                         (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
  const int64_t signedValue = value > 0x7fffffffU ? int64_t(value) - 0x100000000LL : value;
  return static_cast<double>(signedValue) * 1e-6;
}

speed_t baudConstant(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("H30 baud must be 115200, 230400, 460800 or 921600");
  }
}

void ioError(const std::string& action) {
  throw std::system_error(errno, std::generic_category(), action);
}
}  // namespace

bool H30Parser::decode(const uint8_t* payload, size_t size, H30Sample& sample) const {
  bool acc = false, gyro = false, quat = false;
  size_t pos = 0;
  while (pos < size) {
    if (size - pos < 2) return false;
    const uint8_t kind = payload[pos++];
    const size_t length = payload[pos++];
    if (length > size - pos) return false;
    if (((kind == 0x10 || kind == 0x20 || kind == 0x40) && length != 12) ||
        (kind == 0x41 && length != 16) || (kind == 0x01 && length != 2)) return false;
    if (kind == 0x10 || kind == 0x20) {
      auto& vector = kind == 0x10 ? sample.linearAcceleration : sample.angularVelocity;
      for (size_t i = 0; i < 3; ++i) {
        vector[i] = scaledInt32(payload + pos + 4 * i);
        if (kind == 0x20) vector[i] *= 3.14159265358979323846 / 180.;
      }
      if (kind == 0x10) acc = true;
      else gyro = true;
    } else if (kind == 0x41) {
      // Device w,x,y,z -> ROS / hardware_interface x,y,z,w.
      for (size_t i = 0; i < 4; ++i) sample.orientation[(i + 3) % 4] = scaledInt32(payload + pos + 4 * i);
      double norm = 0.;
      for (double v : sample.orientation) norm += v * v;
      norm = std::sqrt(norm);
      if (!std::isfinite(norm) || norm < 0.8 || norm > 1.2) return false;
      for (double& v : sample.orientation) v /= norm;
      quat = true;
    }
    pos += length;
  }
  return acc && gyro && quat;
}

std::vector<H30Sample> H30Parser::feed(const uint8_t* data, size_t size) {
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<H30Sample> samples;
  const uint8_t header[] = {0x59, 0x53};
  while (true) {
    const auto start = std::search(buffer_.begin(), buffer_.end(), std::begin(header), std::end(header));
    if (start == buffer_.end()) {
      const bool keep = !buffer_.empty() && buffer_.back() == 0x59;
      buffer_.clear();
      if (keep) buffer_.push_back(0x59);
      break;
    }
    buffer_.erase(buffer_.begin(), start);
    if (buffer_.size() < 5) break;
    const size_t length = size_t(buffer_[4]) + 7;
    if (buffer_.size() < length) break;
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < length - 2; ++i) {
      a += buffer_[i];
      b += a;
    }
    if (a != buffer_[length - 2] || b != buffer_[length - 1]) {
      ++checksumErrors_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    H30Sample sample;
    sample.tid = uint16_t(buffer_[2]) | (uint16_t(buffer_[3]) << 8);
    if (decode(buffer_.data() + 5, buffer_[4], sample)) {
      ++validFrames_;
      samples.push_back(sample);
    } else {
      ++payloadErrors_;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + length);
  }
  return samples;
}

H30Serial::H30Serial(const std::string& port, int baud) {
  const speed_t speed = baudConstant(baud);
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) ioError("open H30 " + port);
  try {
    if (ioctl(fd_, TIOCEXCL) < 0) ioError("H30 TIOCEXCL");
    if (tcgetattr(fd_, &saved_) < 0) ioError("H30 tcgetattr");
    savedValid_ = true;
    termios settings = saved_;
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | HUPCL);
    settings.c_cflag |= CS8 | CREAD | CLOCAL;
    // VMIN=1 with O_NONBLOCK distinguishes EAGAIN (idle) from EOF (unplug).
    settings.c_cc[VMIN] = 1;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, speed) < 0 || cfsetospeed(&settings, speed) < 0 ||
        tcsetattr(fd_, TCSANOW, &settings) < 0) ioError("configure H30 8N1");
  } catch (...) {
    closePort();
    throw;
  }
}

H30Serial::~H30Serial() { closePort(); }

void H30Serial::closePort() {
  if (fd_ < 0) return;
  if (savedValid_) tcsetattr(fd_, TCSANOW, &saved_);
  ioctl(fd_, TIOCNXCL);
  ::close(fd_);
  fd_ = -1;
}

std::vector<H30Sample> H30Serial::poll() {
  std::vector<H30Sample> samples;
  uint8_t data[4096];
  for (size_t batch = 0; batch < 16; ++batch) {
    const ssize_t count = ::read(fd_, data, sizeof(data));
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      if (errno == EINTR) continue;
      ioError("read H30 (device disconnected?)");
    }
    if (count == 0) throw std::runtime_error("H30 serial EOF / device disconnected");
    auto parsed = parser_.feed(data, static_cast<size_t>(count));
    samples.insert(samples.end(), parsed.begin(), parsed.end());
  }
  return samples;
}

}  // namespace legged
