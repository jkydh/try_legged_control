#include <legged_controllers/TopicImu.h>
#include <gtest/gtest.h>
#include <limits>

namespace {
sensor_msgs::Imu message() {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time(10, 0);
  msg.header.frame_id = "base_imu";
  msg.orientation.w = 1.;
  msg.linear_acceleration.z = 9.81;
  msg.angular_velocity.x = 0.25;
  return msg;
}
}  // namespace

TEST(TopicImu, PreservesUnitsAndNormalizesQuaternion) {
  auto msg = message();
  msg.orientation.w = 1.001;
  auto sample = legged::decodeTopicImu(msg, "base_imu");
  ASSERT_TRUE(sample.valid);
  EXPECT_DOUBLE_EQ(sample.orientation[3], 1.);
  EXPECT_DOUBLE_EQ(sample.angularVelocity[0], 0.25);
  EXPECT_DOUBLE_EQ(sample.linearAcceleration[2], 9.81);
  EXPECT_TRUE(legged::topicImuFresh(sample, ros::Time(10, 1000000), sample.received, 0.1));
}

TEST(TopicImu, RejectsInvalidFieldsAndFrames) {
  auto msg = message();
  EXPECT_FALSE(legged::decodeTopicImu(msg, "different_frame").valid);
  msg.header.stamp = ros::Time();
  EXPECT_FALSE(legged::decodeTopicImu(msg, "base_imu").valid);
  msg = message();
  msg.orientation.w = 0.;
  EXPECT_FALSE(legged::decodeTopicImu(msg, "base_imu").valid);
  msg = message();
  msg.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(legged::decodeTopicImu(msg, "base_imu").valid);
  msg = message();
  msg.orientation_covariance[0] = -1.;
  EXPECT_FALSE(legged::decodeTopicImu(msg, "base_imu").valid);
}

TEST(TopicImu, RejectsStaleReceptionStaleStampAndFutureClock) {
  auto sample = legged::decodeTopicImu(message(), "base_imu");
  EXPECT_FALSE(legged::topicImuFresh(sample, ros::Time(10, 0),
                                    sample.received + std::chrono::milliseconds(101), 0.1));
  EXPECT_FALSE(legged::topicImuFresh(sample, ros::Time(10, 101000000), sample.received, 0.1));
  EXPECT_FALSE(legged::topicImuFresh(sample, ros::Time(9, 0), sample.received, 0.1));
  EXPECT_FALSE(legged::topicImuFresh(sample, ros::Time(), sample.received, 0.1));
}
