#include "legged_hw/H30Imu.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {
template <typename T>
void parameter(ros::NodeHandle& nh, const std::string& name, T& value, const T& fallback) {
  if (!nh.hasParam(name)) value = fallback;
  else if (!nh.getParam(name, value)) throw std::invalid_argument("wrong parameter type: " + name);
}

std::vector<double> diagonal(ros::NodeHandle& nh, const std::string& name) {
  std::vector<double> values;
  parameter(nh, name, values, std::vector<double>{0., 0., 0.});
  if (values.size() != 3) throw std::invalid_argument(name + " must have three diagonal entries");
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.) throw std::invalid_argument(name + " must be finite and nonnegative");
  }
  return values;
}

void run(ros::NodeHandle& nh, ros::NodeHandle& privateNh) {
  std::string port, topic, frame;
  int baud;
  double timeout, startupTimeout, yawOffset;
  parameter<std::string>(privateNh, "port", port, "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31016849-if00");
  parameter<std::string>(privateNh, "topic", topic, "/h30/imu");
  parameter<std::string>(privateNh, "frame_id", frame, "base_imu");
  parameter(privateNh, "baud", baud, 460800);
  parameter(privateNh, "data_timeout", timeout, 0.1);
  parameter(privateNh, "startup_timeout", startupTimeout, 5.0);
  parameter(privateNh, "world_yaw_offset", yawOffset, 0.0);
  if (port.empty() || topic.empty() || frame.empty() || !std::isfinite(timeout) || timeout <= 0. ||
      !std::isfinite(startupTimeout) || startupTimeout <= 0. || !std::isfinite(yawOffset)) {
    throw std::invalid_argument("invalid H30 port/topic/frame/timeout/yaw parameters");
  }
  std::vector<double> mounting;
  parameter(privateNh, "sensor_to_base_rpy", mounting, std::vector<double>{0., 0., 0.});
  if (mounting.size() != 3) throw std::invalid_argument("sensor_to_base_rpy must have three entries in radians");
  for (double v : mounting) if (!std::isfinite(v)) throw std::invalid_argument("non-finite sensor_to_base_rpy");
  tf2::Quaternion baseFromSensor, worldOffset;
  baseFromSensor.setRPY(mounting[0], mounting[1], mounting[2]);
  worldOffset.setRPY(0., 0., yawOffset);
  sensor_msgs::Imu message;
  message.header.frame_id = frame;
  const auto orientationCovariance = diagonal(privateNh, "orientation_covariance_diagonal");
  const auto gyroCovariance = diagonal(privateNh, "angular_velocity_covariance_diagonal");
  const auto accCovariance = diagonal(privateNh, "linear_acceleration_covariance_diagonal");
  for (size_t i = 0; i < 3; ++i) {
    message.orientation_covariance[i * 4] = orientationCovariance[i];
    message.angular_velocity_covariance[i * 4] = gyroCovariance[i];
    message.linear_acceleration_covariance[i * 4] = accCovariance[i];
  }
  auto publisher = nh.advertise<sensor_msgs::Imu>(topic, 10, false);
  legged::H30Serial serial(port, baud);
  ROS_INFO_STREAM("H30 receive-only: " << port << " @ " << baud << " -> " << topic << " frame=" << frame);
  using Clock = std::chrono::steady_clock;
  auto lastSample = Clock::now();
  bool received = false;
  ros::WallRate rate(1000.);
  while (ros::ok()) {
    const auto samples = serial.poll();
    for (const auto& sample : samples) {
      const tf2::Quaternion sensorOrientation(sample.orientation[0], sample.orientation[1],
                                               sample.orientation[2], sample.orientation[3]);
      auto orientation = worldOffset * sensorOrientation * baseFromSensor.inverse();
      orientation.normalize();
      const auto angular = tf2::quatRotate(baseFromSensor, tf2::Vector3(sample.angularVelocity[0],
                                          sample.angularVelocity[1], sample.angularVelocity[2]));
      const auto acceleration = tf2::quatRotate(baseFromSensor, tf2::Vector3(sample.linearAcceleration[0],
                                               sample.linearAcceleration[1], sample.linearAcceleration[2]));
      message.header.stamp = ros::Time::now();  // Host receive time, not a sensor sampling clock.
      message.header.seq = sample.tid;
      message.orientation.x = orientation.x();
      message.orientation.y = orientation.y();
      message.orientation.z = orientation.z();
      message.orientation.w = orientation.w();
      message.angular_velocity.x = angular.x();
      message.angular_velocity.y = angular.y();
      message.angular_velocity.z = angular.z();
      message.linear_acceleration.x = acceleration.x();
      message.linear_acceleration.y = acceleration.y();
      message.linear_acceleration.z = acceleration.z();
      publisher.publish(message);
      lastSample = Clock::now();
      received = true;
    }
    if (std::chrono::duration<double>(Clock::now() - lastSample).count() > (received ? timeout : startupTimeout)) {
      throw std::runtime_error("H30 timed out: no complete acc/gyro/quaternion frame; check baud/output configuration");
    }
    ROS_INFO_STREAM_THROTTLE(5., "H30 complete_frames=" << serial.parser().validFrames()
                             << " checksum_errors=" << serial.parser().checksumErrors()
                             << " incomplete_or_bad_payload=" << serial.parser().payloadErrors());
    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "h30_imu");
  ros::NodeHandle nh, privateNh("~");
  try {
    run(nh, privateNh);
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("H30 stopped: " << error.what());
    return 1;
  }
  return 0;
}
