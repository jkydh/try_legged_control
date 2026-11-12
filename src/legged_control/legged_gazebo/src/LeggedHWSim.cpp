/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#include "legged_gazebo/LeggedHWSim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <legged_common/hardware_interface/HybridJointTorque.h>

namespace legged {
bool LeggedHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
                          const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) {
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  if (!ret) return false;
  double configuredEffortLimit;
  model_nh.param("gazebo/effort_limit", configuredEffortLimit, std::numeric_limits<double>::max());
  if (!std::isfinite(configuredEffortLimit) || configuredEffortLimit <= 0.0) {
    ROS_ERROR("gazebo/effort_limit must be finite and positive");
    return false;
  }
  // Joint interface
  registerInterface(&hybridJointInterface_);
  std::vector<std::string> names = ej_interface_.getNames();
  for (const auto& name : names) {
    const auto joint = urdf_model->getJoint(name);
    if (!joint || !joint->limits || !std::isfinite(joint->limits->effort) || joint->limits->effort <= 0.0) {
      ROS_ERROR_STREAM("No valid URDF effort limit for " << name);
      return false;
    }
    hybridJointDatas_.push_back(HybridJointData{.joint_ = ej_interface_.getHandle(name),
                                               .effortLimit_ = std::min(joint->limits->effort, configuredEffortLimit)});
    HybridJointData& back = hybridJointDatas_.back();
    hybridJointInterface_.registerHandle(HybridJointHandle(back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
    cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<HybridJointCommand>()));
  }
  torqueAudit_.name = names;
  const size_t count = names.size();
  torqueAudit_.limit.resize(count);
  torqueAudit_.position_term.resize(count);
  torqueAudit_.velocity_term.resize(count);
  torqueAudit_.feedforward.resize(count);
  torqueAudit_.requested_effort.resize(count);
  torqueAudit_.applied_effort.resize(count);
  torqueAudit_.measured_effort.resize(count);
  torqueAudit_.phase.resize(count);
  torqueAudit_.peak_requested_effort.resize(count);
  torqueAudit_.peak_applied_effort.resize(count);
  torqueAudit_.peak_measured_effort.resize(count);
  torqueAudit_.saturation_count.resize(count);
  torqueAudit_.peak_time.resize(count);
  torqueAudit_.peak_position_term.resize(count);
  torqueAudit_.peak_velocity_term.resize(count);
  torqueAudit_.peak_feedforward.resize(count);
  torqueAudit_.peak_measured_velocity.resize(count);
  torqueAudit_.peak_phase.resize(count);
  bool publishTorqueDiagnostics = false;
  model_nh.param("gazebo/torque_diagnostics", publishTorqueDiagnostics, false);
  if (publishTorqueDiagnostics) {
    torquePublisher_.reset(new realtime_tools::RealtimePublisher<legged_gazebo::TorqueDiagnostics>(model_nh, "torque_diagnostics", 1, true));
    torquePublisher_->msg_ = torqueAudit_;
  }
  // IMU interface
  registerInterface(&imuSensorInterface_);
  XmlRpc::XmlRpcValue xmlRpcValue;
  if (!model_nh.getParam("gazebo/imus", xmlRpcValue)) {
    ROS_WARN("No imu specified");
  } else {
    parseImu(xmlRpcValue, parent_model);
  }
  if (!model_nh.getParam("gazebo/delay", delay_)) {
    delay_ = 0.;
  }
  if (!model_nh.getParam("gazebo/contacts", xmlRpcValue)) {
    ROS_WARN("No contacts specified");
  } else {
    parseContacts(xmlRpcValue);
  }

  contactManager_ = parent_model->GetWorld()->Physics()->GetContactManager();
  contactManager_->SetNeverDropContacts(true);  // NOTE: If false, we need to select view->contacts in gazebo GUI to
                                                // avoid returning nothing when calling ContactManager::GetContacts()
  return ret;
}

void LeggedHWSim::readSim(ros::Time time, ros::Duration period) {
  //  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period); The DefaultRobotHWSim Provide a bias joint velocity
  for (unsigned int j = 0; j < n_dof_; j++) {
    double position = sim_joints_[j]->Position(0);

    const double delta = joint_types_[j] == urdf::Joint::PRISMATIC
                             ? position - joint_position_[j]
                             : angles::shortest_angular_distance(joint_position_[j], position);
    joint_velocity_[j] = period.toSec() > 0 ? delta / period.toSec() : 0.0;
    if (firstRead_ || time == ros::Time(period.toSec())) {
      joint_velocity_[j] = 0;
    }
    if (joint_types_[j] == urdf::Joint::CONTINUOUS && !firstRead_) {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
    } else {
      // Bounded revolute joints must use the URDF/Gazebo angle branch.
      // Integrating from DefaultRobotHWSim's initial position can add 2*pi
      // to a deeply flexed knee and make PD command the wrong rotation.
      joint_position_[j] = position;
    }
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }
  firstRead_ = false;

  // Imu Sensor
  for (auto& imu : imuDatas_) {
    // TODO(qiayuan) Add noise
    ignition::math::Pose3d pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X();
    imu.ori_[1] = pose.Rot().Y();
    imu.ori_[2] = pose.Rot().Z();
    imu.ori_[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = rate.X();
    imu.angularVel_[1] = rate.Y();
    imu.angularVel_[2] = rate.Z();

    ignition::math::Vector3d gravity = {0., 0., -9.81};
    ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = accel.X();
    imu.linearAcc_[1] = accel.Y();
    imu.linearAcc_[2] = accel.Z();
  }

  // Contact Sensor
  for (auto& state : name2contact_) {
    state.second = false;
  }
  for (const auto& contact : contactManager_->GetContacts()) {
    if (static_cast<uint32_t>(contact->time.sec) != (time - period).sec ||
        static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec) {
      continue;
    }
    std::string linkName = contact->collision1->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end()) {
      name2contact_[linkName] = true;
    }
    linkName = contact->collision2->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end()) {
      name2contact_[linkName] = true;
    }
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_) {
    cmd = 0;
  }
  for (auto& cmd : joint_velocity_command_) {
    cmd = 0;
  }
  for (auto& joint : hybridJointDatas_) {
    joint.posDes_ = joint.joint_.getPosition();
    joint.velDes_ = joint.joint_.getVelocity();
    joint.kp_ = 0.;
    joint.kd_ = 0.;
    joint.ff_ = 0.;
  }
}

void LeggedHWSim::writeSim(ros::Time time, ros::Duration period) {
  size_t index = 0;
  ++torqueAudit_.samples;
  for (auto joint : hybridJointDatas_) {
    auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
    if (time == ros::Time(period.toSec())) {  // Simulation reset
      buffer.clear();
    }

    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
      buffer.pop_back();
    }
    buffer.push_front(HybridJointCommand{
        .stamp_ = time, .posDes_ = joint.posDes_, .velDes_ = joint.velDes_, .kp_ = joint.kp_, .kd_ = joint.kd_, .ff_ = joint.ff_});

    const auto& cmd = buffer.back();
    // Limit the SUM, including feedback, before DefaultRobotHWSim/URDF limits.
    const auto torque = computeHybridJointTorque(cmd.posDes_, cmd.velDes_, cmd.kp_, cmd.kd_, cmd.ff_,
                                                joint.joint_.getPosition(), joint.joint_.getVelocity(), joint.effortLimit_);
    const double positionTerm = torque.position, velocityTerm = torque.velocity;
    const double requested = torque.requested, applied = torque.applied;
    joint.joint_.setCommand(applied);
    torqueAudit_.limit[index] = joint.effortLimit_;
    torqueAudit_.position_term[index] = positionTerm;
    torqueAudit_.velocity_term[index] = velocityTerm;
    torqueAudit_.feedforward[index] = cmd.ff_;
    torqueAudit_.requested_effort[index] = requested;
    torqueAudit_.applied_effort[index] = applied;
    torqueAudit_.measured_effort[index] = joint.joint_.getEffort();
    torqueAudit_.phase[index] = cmd.kp_ > 0.0 ? 1 : (cmd.kd_ > 0.0 || cmd.ff_ != 0.0 ? 2 : 0);
    torqueAudit_.peak_applied_effort[index] = std::max(torqueAudit_.peak_applied_effort[index], std::abs(applied));
    torqueAudit_.peak_measured_effort[index] = std::max(torqueAudit_.peak_measured_effort[index], std::abs(joint.joint_.getEffort()));
    if (std::abs(requested) > torqueAudit_.peak_requested_effort[index]) {
      torqueAudit_.peak_requested_effort[index] = std::abs(requested);
      torqueAudit_.peak_time[index] = time.toSec();
      torqueAudit_.peak_position_term[index] = positionTerm;
      torqueAudit_.peak_velocity_term[index] = velocityTerm;
      torqueAudit_.peak_feedforward[index] = cmd.ff_;
      torqueAudit_.peak_measured_velocity[index] = joint.joint_.getVelocity();
      torqueAudit_.peak_phase[index] = torqueAudit_.phase[index];
    }
    if (!std::isfinite(requested) || std::abs(requested) > joint.effortLimit_) {
      ++torqueAudit_.saturation_count[index];
    }
    ++index;
  }
  DefaultRobotHWSim::writeSim(time, period);
  if (torquePublisher_ && (time - lastTorquePublish_).toSec() >= 0.01 && torquePublisher_->trylock()) {
    lastTorquePublish_ = time;
    torqueAudit_.header.stamp = time;
    torquePublisher_->msg_ = torqueAudit_;
    torquePublisher_->unlockAndPublish();
  }
}

void LeggedHWSim::parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel) {
  ROS_ASSERT(imuDatas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imuDatas.begin(); it != imuDatas.end(); ++it) {
    if (!it->second.hasMember("frame_id")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    } else if (!it->second.hasMember("orientation_covariance_diagonal")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    } else if (!it->second.hasMember("angular_velocity_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    } else if (!it->second.hasMember("linear_acceleration_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    XmlRpc::XmlRpcValue oriCov = imuDatas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(oriCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(oriCov.size() == 3);
    for (int i = 0; i < oriCov.size(); ++i) {
      ROS_ASSERT(oriCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue angularCov = imuDatas[it->first]["angular_velocity_covariance"];
    ROS_ASSERT(angularCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angularCov.size() == 3);
    for (int i = 0; i < angularCov.size(); ++i) {
      ROS_ASSERT(angularCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue linearCov = imuDatas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linearCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linearCov.size() == 3);
    for (int i = 0; i < linearCov.size(); ++i) {
      ROS_ASSERT(linearCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    std::string frameId = imuDatas[it->first]["frame_id"];
    gazebo::physics::LinkPtr linkPtr = parentModel->GetLink(frameId);
    ROS_ASSERT(linkPtr != nullptr);
    imuDatas_.push_back((ImuData{
        .linkPtr_ = linkPtr,
        .ori_ = {0., 0., 0., 0.},
        .oriCov_ = {static_cast<double>(oriCov[0]), 0., 0., 0., static_cast<double>(oriCov[1]), 0., 0., 0., static_cast<double>(oriCov[2])},
        .angularVel_ = {0., 0., 0.},
        .angularVelCov_ = {static_cast<double>(angularCov[0]), 0., 0., 0., static_cast<double>(angularCov[1]), 0., 0., 0.,
                           static_cast<double>(angularCov[2])},
        .linearAcc_ = {0., 0., 0.},
        .linearAccCov_ = {static_cast<double>(linearCov[0]), 0., 0., 0., static_cast<double>(linearCov[1]), 0., 0., 0.,
                          static_cast<double>(linearCov[2])}}));
    ImuData& imuData = imuDatas_.back();
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(it->first, frameId, imuData.ori_, imuData.oriCov_,
                                                                           imuData.angularVel_, imuData.angularVelCov_, imuData.linearAcc_,
                                                                           imuData.linearAccCov_));
  }
}

void LeggedHWSim::parseContacts(XmlRpc::XmlRpcValue& contactNames) {
  ROS_ASSERT(contactNames.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < contactNames.size(); ++i) {  // NOLINT(modernize-loop-convert)
    std::string name = contactNames[i];
    name2contact_.insert(std::make_pair(name, false));
    contactSensorInterface_.registerHandle(ContactSensorHandle(name, &name2contact_[name]));
  }
  registerInterface(&contactSensorInterface_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
