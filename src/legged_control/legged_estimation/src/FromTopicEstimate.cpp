//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/FromTopiceEstimate.h"
#include <ros/callback_queue.h>
#include <stdexcept>

namespace legged {
FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
  ros::NodeHandle nh;
  // A controller can start on the first physics tick. Do not let an empty
  // odometry buffer turn the initial MPC target into a pose at ground level.
  // Use a private queue and wall timeout because simulation may be paused.
  ros::CallbackQueue queue;
  ros::NodeHandle initialNh;
  initialNh.setCallbackQueue(&queue);
  nav_msgs::Odometry::ConstPtr initial;
  auto initialSub = initialNh.subscribe<nav_msgs::Odometry>(
      "/ground_truth/state", 1, [&initial](const nav_msgs::Odometry::ConstPtr& msg) { initial = msg; });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (!initial && ros::ok() && ros::WallTime::now() < deadline) {
    queue.callAvailable(ros::WallDuration(0.01));
  }
  if (!initial) {
    throw std::runtime_error("No initial /ground_truth/state received for cheater estimator");
  }
  buffer_.initRT(*initial);
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  updateAngular(quatToZyx(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  publishMsgs(odom);

  return rbdState_;
}

}  // namespace legged
