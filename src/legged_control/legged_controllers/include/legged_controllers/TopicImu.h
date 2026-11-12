#pragma once

#include <sensor_msgs/Imu.h>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

namespace legged {

struct TopicImuSample {
  std::array<double, 4> orientation{};  // x, y, z, w
  std::array<double, 3> angularVelocity{}, linearAcceleration{};
  std::array<double, 9> orientationCovariance{}, angularVelocityCovariance{}, linearAccelerationCovariance{};
  ros::Time stamp;
  std::chrono::steady_clock::time_point received;
  bool valid = false;
};

inline TopicImuSample decodeTopicImu(const sensor_msgs::Imu& msg, const std::string& frame) {
  TopicImuSample sample;
  sample.received = std::chrono::steady_clock::now();
  sample.stamp = msg.header.stamp;
  if (msg.header.frame_id != frame || sample.stamp.isZero()) return sample;
  sample.orientation = {{msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w}};
  sample.angularVelocity = {{msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z}};
  sample.linearAcceleration = {{msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z}};
  for (size_t i = 0; i < 9; ++i) {
    sample.orientationCovariance[i] = msg.orientation_covariance[i];
    sample.angularVelocityCovariance[i] = msg.angular_velocity_covariance[i];
    sample.linearAccelerationCovariance[i] = msg.linear_acceleration_covariance[i];
  }
  for (double v : sample.orientation) if (!std::isfinite(v)) return sample;
  for (double v : sample.angularVelocity) if (!std::isfinite(v)) return sample;
  for (double v : sample.linearAcceleration) if (!std::isfinite(v)) return sample;
  for (size_t i = 0; i < 9; ++i) {
    if (!std::isfinite(sample.orientationCovariance[i]) || !std::isfinite(sample.angularVelocityCovariance[i]) ||
        !std::isfinite(sample.linearAccelerationCovariance[i])) return sample;
    if (i % 4 == 0 && (sample.orientationCovariance[i] < 0. || sample.angularVelocityCovariance[i] < 0. ||
                      sample.linearAccelerationCovariance[i] < 0.)) return sample;
  }
  double norm = 0.;
  for (double v : sample.orientation) norm += v * v;
  norm = std::sqrt(norm);
  if (norm < 0.8 || norm > 1.2) return sample;
  for (double& v : sample.orientation) v /= norm;
  sample.valid = true;
  return sample;
}

inline bool topicImuFresh(const TopicImuSample& sample, const ros::Time& now,
                          std::chrono::steady_clock::time_point wallNow, double timeout) {
  if (!sample.valid || now.isZero()) return false;
  const double receiptAge = std::chrono::duration<double>(wallNow - sample.received).count();
  const double stampAge = (now - sample.stamp).toSec();
  // Both processes must share a ROS clock. Steady time detects missing data
  // even while /clock is paused; old messages are never silently reused.
  return receiptAge >= 0. && receiptAge <= timeout && stampAge >= -0.02 && stampAge <= timeout;
}

}  // namespace legged
