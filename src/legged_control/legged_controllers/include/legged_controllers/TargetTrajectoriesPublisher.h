//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
using namespace ocs2;

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        tf2_(buffer_) {
    ros::NodeHandle privateNh("~");
    privateNh.param("cmd_vel_timeout", cmdVelTimeout_, 0.0);
    privateNh.param("require_mpc_ready", requireMpcReady_, false);
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      const auto observation = readObservation();
      if (observation.time == 0.0 || (requireMpcReady_ && !mpcReady_)) {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      velocityCommandActive_ = false;
      const auto trajectories = goalToTargetTrajectories_(cmdGoal, observation);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      const auto observation = readObservation();
      if (observation.time == 0.0 || (requireMpcReady_ && !mpcReady_)) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      if (!cmdVel.allFinite()) return;
      lastCmdVelTime_ = ros::Time::now();
      velocityCommandActive_ = !cmdVel.isZero(1e-9);

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, observation);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
    readySub_ = nh.subscribe<std_msgs::Bool>(topicPrefix + "_mpc_ready", 1,
                                           [this](const std_msgs::Bool::ConstPtr& msg) { mpcReady_ = msg->data; });
    if (cmdVelTimeout_ > 0.0) {
      commandTimer_ = nh.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
        if (!velocityCommandActive_ || (ros::Time::now() - lastCmdVelTime_).toSec() < cmdVelTimeout_) return;
        velocityCommandActive_ = false;
        const auto observation = readObservation();
        if (observation.time != 0.0 && (!requireMpcReady_ || mpcReady_)) {
          targetTrajectoriesPublisher_->publishTargetTrajectories(cmdVelToTargetTrajectories_(vector_t::Zero(4), observation));
          ROS_WARN("cmd_vel timed out; requested zero velocity. Select stance after the robot has slowed down.");
        }
      });
    }
  }

 private:
  SystemObservation readObservation() const {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    return latestObservation_;
  }
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_, readySub_;
  ros::Timer commandTimer_;
  ros::Time lastCmdVelTime_;
  double cmdVelTimeout_ = 0.0;
  bool velocityCommandActive_ = false, requireMpcReady_ = false, mpcReady_ = false;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
