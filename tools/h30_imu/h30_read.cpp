// Standalone Linux/Jetson UART reader using the same driver as the ROS controller.
#include "legged_hw/H30Imu.h"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }
using Clock = std::chrono::steady_clock;

struct Options {
  std::string port = "/dev/ttyTHS1", csv;
  int baud = 460800;
  double duration = 0., startupTimeout = 5., dataTimeout = 1.;
  bool requireComplete = false;
};

double number(const std::string& value, bool allowZero = false) {
  size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result) || result < 0. || (!allowZero && result == 0.)) {
    throw std::invalid_argument("Invalid numeric argument: " + value);
  }
  return result;
}

Options options(int argc, char** argv) {
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--require-complete") { result.requireComplete = true; continue; }
    if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--port") result.port = value;
    else if (key == "--csv") result.csv = value;
    else if (key == "--baud") {
      const double baud = number(value);
      if (baud != 115200 && baud != 230400 && baud != 460800 && baud != 921600) {
        throw std::invalid_argument("Supported baud rates: 115200, 230400, 460800, 921600");
      }
      result.baud = static_cast<int>(baud);
    } else if (key == "--duration") result.duration = number(value, true);
    else if (key == "--startup-timeout") result.startupTimeout = number(value);
    else if (key == "--data-timeout") result.dataTimeout = number(value);
    else throw std::invalid_argument("Unknown argument: " + key);
  }
  if (result.port.empty()) throw std::invalid_argument("Port must not be empty");
  return result;
}

// Exclusive creation avoids silently overwriting an earlier recording.
class Csv {
 public:
  explicit Csv(const std::string& path) {
    if (path.empty()) return;
    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
    if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "create CSV " + path);
  }
  ~Csv() { if (fd_ >= 0) ::close(fd_); }
  void write(const std::string& row) {
    if (fd_ < 0) return;
    size_t done = 0;
    while (done < row.size()) {
      const auto count = ::write(fd_, row.data() + done, row.size() - done);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::system_error(count < 0 ? errno : EIO, std::generic_category(), "write CSV");
      done += static_cast<size_t>(count);
    }
  }
 private:
  int fd_ = -1;
};

template <size_t N>
void columns(std::ostream& out, const std::array<double, N>& values, bool present) {
  for (double value : values) {
    out << ',';
    if (present) out << value;
  }
}

int run(const Options& config) {
  legged::H30Serial serial(config.port, config.baud, config.requireComplete);
  Csv csv(config.csv);
  csv.write("time_unix_s,elapsed_s,tid,acc_x_m_s2,acc_y_m_s2,acc_z_m_s2,"
            "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,quat_x,quat_y,quat_z,quat_w,"
            "roll_rad,pitch_rad,yaw_rad,temp_C,complete\n");
  std::cout << "OPEN: " << config.port << " " << config.baud << " baud 8N1, no flow control; receive-only\n"
            << "Units: acceleration=m/s^2 (gravity retained), gyro=rad/s, rpy=rad, quaternion=x,y,z,w\n"
            << "Mode: " << (config.requireComplete ? "complete acc/gyro/quaternion frames" : "diagnostic per-frame fields")
            << std::endl;
  const auto start = Clock::now();
  auto lastImu = start, nextPrint = start;
  size_t imuFrames = 0, completeFrames = 0;
  bool timeout = false;
  while (!interrupted) {
    for (const auto& sample : serial.poll()) {
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      if (sample.hasImuData()) { lastImu = now; ++imuFrames; }
      if (sample.complete()) ++completeFrames;
      const double unixTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      std::ostringstream row;
      row << std::setprecision(17) << unixTime << ',' << elapsed << ',' << sample.tid;
      columns(row, sample.linearAcceleration, sample.hasLinearAcceleration);
      columns(row, sample.angularVelocity, sample.hasAngularVelocity);
      columns(row, sample.orientation, sample.hasOrientation);
      columns(row, sample.rpy, sample.hasRpy);
      row << ',';
      if (sample.hasTemperature) row << sample.temperature;
      row << ',' << sample.complete() << '\n';
      csv.write(row.str());
      if (now >= nextPrint && sample.hasImuData()) {
        std::cout << std::fixed << std::setprecision(6) << "tid=" << sample.tid;
        auto show = [](const char* label, const auto& values, bool present) {
          if (!present) return;
          std::cout << ' ' << label << "=[";
          for (size_t i = 0; i < values.size(); ++i) std::cout << (i ? "," : "") << values[i];
          std::cout << ']';
        };
        show("acc_m_s2", sample.linearAcceleration, sample.hasLinearAcceleration);
        show("gyro_rad_s", sample.angularVelocity, sample.hasAngularVelocity);
        show("quat_xyzw", sample.orientation, sample.hasOrientation);
        show("rpy_rad", sample.rpy, sample.hasRpy);
        if (sample.hasTemperature) std::cout << " temp_C=" << sample.temperature;
        std::cout << " complete=" << sample.complete() << std::endl;
        nextPrint = now + std::chrono::milliseconds(200);
      }
    }
    const auto now = Clock::now();
    if (std::chrono::duration<double>(now - lastImu).count() > (imuFrames ? config.dataTimeout : config.startupTimeout)) {
      timeout = true;
      std::cerr << "FAIL: " << (imuFrames ? "IMU stream timed out" : "No accepted IMU frame before startup timeout")
                << "; check port, RX/TX, common ground, pinmux, baud and enabled output fields.\n";
      break;
    }
    if (config.duration > 0. && std::chrono::duration<double>(now - start).count() >= config.duration) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto& parser = serial.parser();
  std::cout << "SUMMARY: bytes=" << serial.bytesReceived() << " accepted_frames=" << parser.validFrames()
            << " imu_frames=" << imuFrames << " complete_frames=" << completeFrames
            << " checksum_errors=" << parser.checksumErrors() << " payload_errors=" << parser.payloadErrors()
            << " imu_hz=" << imuFrames / std::chrono::duration<double>(Clock::now() - start).count() << std::endl;
  if (interrupted) return 130;
  if (timeout || imuFrames == 0) return 1;
  std::cout << "PASS: received checked IMU frames" << std::endl;
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help") {
      std::cout << "h30_read [--port /dev/ttyTHS1] [--baud 460800] [--duration seconds]\n"
                   "         [--csv new-file.csv] [--startup-timeout 5] [--data-timeout 1]\n"
                   "         [--require-complete]\n"
                   "Default: continuous diagnostic reading until Ctrl+C. Missing fields stay empty.\n"
                   "Use --require-complete to require acc+gyro+quaternion in each accepted frame.\n";
      return 0;
    }
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try { return run(options(argc, argv)); }
  catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
