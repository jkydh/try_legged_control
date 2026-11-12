//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/model.hpp>
#include <stdexcept>
#include <algorithm>
#include <boost/property_tree/info_parser.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  std::string imuSource, imuTopic;
  controller_nh.param<std::string>("imu_source", imuSource, "hardware");
  controller_nh.param<std::string>("imu_topic", imuTopic, "/h30/imu");
  controller_nh.param<std::string>("imu_frame_id", imuFrameId_, "base_imu");
  controller_nh.param("imu_timeout", imuTimeout_, 0.1);
  if ((imuSource != "hardware" && imuSource != "topic") || imuTopic.empty() || imuFrameId_.empty() ||
      !std::isfinite(imuTimeout_) || imuTimeout_ <= 0.) {
    ROS_ERROR("Invalid imu_source/topic/frame_id/timeout parameters");
    return false;
  }
  useTopicImu_ = imuSource == "topic";
  if (useTopicImu_) {
    imuSubscriber_ = controller_nh.subscribe(imuTopic, 1, &LeggedController::imuCallback, this,
                                            ros::TransportHints().tcpNoDelay());
    imuUsedPublisher_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(controller_nh, "imu_used", 1));
    imuUsedPublisher_->msg_.header.frame_id = imuFrameId_;
    ROS_INFO_STREAM("LeggedController IMU source=topic " << imuTopic << " timeout=" << imuTimeout_);
  }
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  if (!controller_nh.getParam("/urdfFile", urdfFile) || !controller_nh.getParam("/taskFile", taskFile) ||
      !controller_nh.getParam("/referenceFile", referenceFile)) {
    ROS_ERROR("Missing urdfFile, taskFile or referenceFile parameter");
    return false;
  }
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  const auto& model = leggedInterface_->getPinocchioInterface().getModel();
  hybridJointHandles_.resize(leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
  for (size_t i = 1; i < model.joints.size(); ++i) {
    if (model.idx_qs[i] < 6) continue;  // Floating base.
    if (model.nqs[i] != 1 || model.nvs[i] != 1) {
      ROS_ERROR("LeggedController requires single-DoF actuated joints");
      return false;
    }
    hybridJointHandles_.at(model.idx_qs[i] - 6) = hybridJointInterface->getHandle(model.names[i]);
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  boost::property_tree::ptree settings;
  boost::property_tree::read_info(taskFile, settings);
  standupEnabled_ = settings.get<bool>("standup.enabled", false);
  standupDuration_ = settings.get<scalar_t>("standup.duration", 3.0);
  standupKp_ = settings.get<scalar_t>("standup.kp", 60.0);
  standupKd_ = settings.get<scalar_t>("standup.kd", 3.0);
  jointVelocityGain_ = settings.get<scalar_t>("wbc.jointVelocityGain", 3.0);
  if (!std::isfinite(jointVelocityGain_) || jointVelocityGain_ < 0.0) {
    ROS_ERROR("Invalid WBC joint velocity gain");
    return false;
  }
  if (standupDuration_ <= 0 || standupKp_ <= 0 || standupKd_ < 0) {
    ROS_ERROR("Invalid standup duration or gains");
    return false;
  }
  standupTargetPosition_.resize(hybridJointHandles_.size());
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", standupTargetPosition_);
  if (standupEnabled_) loadData::loadCppDataType(referenceFile, "comHeight", standupHeight_);

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  mpcRunning_ = false;
  publishMpcReady(false);
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  imuStartupFailed_ = false;
  try {
    updateStateEstimation(time, ros::Duration(0.002));
  } catch (const std::exception& error) {
    // controller_manager marks RUNNING after starting() returns. Latch this
    // failure so the first update stops it instead of entering stand-up/MPC.
    imuStartupFailed_ = true;
    ROS_ERROR_STREAM("LeggedController cannot start: " << error.what());
    stopping(time);
    return;
  }
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  standingUp_ = standupEnabled_;
  if (standingUp_) {
    standupStartTime_ = time;
    standupReadySince_ = ros::Time();
    standupStartPosition_ = centroidal_model::getJointAngles(currentObservation_.state,
                                                            leggedInterface_->getCentroidalModelInfo());
    ROS_INFO("Stand-up transition started; MPC/WBC will engage after the posture settles.");
    return;
  }
  initializePolicy();
}

void LeggedController::initializePolicy() {
  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
  if (standupEnabled_) {
    vector_t target = currentObservation_.state;
    target.head(6).setZero();
    target(8) = standupHeight_;
    target.segment<2>(10).setZero();
    target.tail(standupTargetPosition_.size()) = standupTargetPosition_;
    target_trajectories = TargetTrajectories({currentObservation_.time, currentObservation_.time + 1.0},
                                           {currentObservation_.state, target},
                                           {currentObservation_.input, currentObservation_.input});
  }

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
  publishMpcReady(true);
}

bool LeggedController::updateStandup(const ros::Time& time) {
  const scalar_t elapsed = (time - standupStartTime_).toSec();
  const scalar_t s = std::min(1.0, std::max(0.0, elapsed / standupDuration_));
  const scalar_t blend = s * s * (3.0 - 2.0 * s);
  const scalar_t rate = 6.0 * s * (1.0 - s) / standupDuration_;
  const vector_t displacement = standupTargetPosition_ - standupStartPosition_;
  const vector_t desired = standupStartPosition_ + blend * displacement;
  for (size_t j = 0; j < hybridJointHandles_.size(); ++j) {
    hybridJointHandles_[j].setCommand(desired(j), rate * displacement(j), standupKp_, standupKd_, 0.0);
  }

  const auto& info = leggedInterface_->getCentroidalModelInfo();
  const vector_t jointError = centroidal_model::getJointAngles(currentObservation_.state, info) - standupTargetPosition_;
  const bool ready = elapsed >= standupDuration_ && jointError.cwiseAbs().maxCoeff() < 0.15 &&
                     currentObservation_.state(8) > 0.24 && std::abs(currentObservation_.state(10)) < 0.25 &&
                     std::abs(currentObservation_.state(11)) < 0.25;
  if (!ready) standupReadySince_ = ros::Time();
  if (ready && standupReadySince_.isZero()) standupReadySince_ = time;
  if (ready && (time - standupReadySince_).toSec() >= 0.5) {
    standingUp_ = false;
    ROS_INFO("Stand-up posture settled; handing over to MPC + WBC.");
    initializePolicy();
    return false;
  }
  if (elapsed > standupDuration_ + 5.0) {
    throw std::runtime_error("Stand-up did not reach a stable posture within the timeout");
  }
  return true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  try {
    if (imuStartupFailed_) throw std::runtime_error("IMU unavailable at controller start; restart after valid data arrives");
    // State Estimate
    updateStateEstimation(time, period);

    if (!measuredRbdState_.allFinite() || period.toSec() <= 0) {
      throw std::runtime_error("Invalid measured state or control period");
    }
    if (standingUp_) {
      if (!safetyChecker_->check(currentObservation_, currentObservation_.state, currentObservation_.input)) {
        throw std::runtime_error("Unsafe orientation during stand-up");
      }
      if (updateStandup(time)) {
        observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
        return;
      }
    }

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    if (period.toSec() <= 0 || !measuredRbdState_.allFinite() ||
        !safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
      throw std::runtime_error("Invalid state, policy or control period");
    }

    // Whole body control
    currentObservation_.input = optimizedInput;

    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
    wbc_->setTorqueOffset(jointVelocityGain_ * (velDes - measuredRbdState_.tail(hybridJointHandles_.size())));

    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    wbcTimer_.endTimer();

    vector_t torque = x.tail(leggedInterface_->getCentroidalModelInfo().actuatedDofNum);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());

    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, jointVelocityGain_, torque(j));
    }

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
    selfCollisionVisualization_->update(currentObservation_);

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("[LeggedController] Stopping: " << error.what());
    stopping(time);
    stopRequest(time);
    return;
  }
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  const TopicImuSample* topicImu = nullptr;
  if (useTopicImu_) {
    topicImu = imuBuffer_.readFromRT();
    if (!topicImuFresh(*topicImu, time, std::chrono::steady_clock::now(), imuTimeout_)) {
      throw std::runtime_error("External IMU missing, invalid, wrong frame/clock or stale; no fallback to simulation");
    }
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = topicImu ? topicImu->orientation[i] : imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = topicImu ? topicImu->angularVelocity[i] : imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = topicImu ? topicImu->linearAcceleration[i] : imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = topicImu ? topicImu->orientationCovariance[i] : imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = topicImu ? topicImu->angularVelocityCovariance[i] : imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = topicImu ? topicImu->linearAccelerationCovariance[i] : imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  if (topicImu && imuUsedPublisher_->trylock()) {
    auto& message = imuUsedPublisher_->msg_;
    message.header.stamp = topicImu->stamp;
    message.orientation.x = quat.x();
    message.orientation.y = quat.y();
    message.orientation.z = quat.z();
    message.orientation.w = quat.w();
    message.angular_velocity.x = angularVel.x();
    message.angular_velocity.y = angularVel.y();
    message.angular_velocity.z = angularVel.z();
    message.linear_acceleration.x = linearAccel.x();
    message.linear_acceleration.y = linearAccel.y();
    message.linear_acceleration.z = linearAccel.z();
    for (size_t i = 0; i < 9; ++i) {
      message.orientation_covariance[i] = topicImu->orientationCovariance[i];
      message.angular_velocity_covariance[i] = topicImu->angularVelocityCovariance[i];
      message.linear_acceleration_covariance[i] = topicImu->linearAccelerationCovariance[i];
    }
    imuUsedPublisher_->unlockAndPublish();
  }
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  imuSubscriber_.shutdown();
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  mpcReadyPublisher_ = nh.advertise<std_msgs::Bool>(robotName + "_mpc_ready", 1, true);
  publishMpcReady(false);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
  imuBuffer_.writeFromNonRT(decodeTopicImu(*message, imuFrameId_));
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  if (useTopicImu_) throw std::runtime_error("H30 topic requires cheater:=false; cheater overwrites IMU attitude");
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
