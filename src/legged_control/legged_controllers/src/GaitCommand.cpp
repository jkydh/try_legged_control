#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

// Uses the same gait.info and transport as the original keyboard gait publisher.
class GaitCommand {
 public:
  GaitCommand(ros::NodeHandle& nh, ros::NodeHandle& privateNh) {
    std::string file;
    if (!nh.getParam("/gaitCommandFile", file)) throw std::runtime_error("Missing /gaitCommandFile");
    stance_ = ocs2::legged_robot::createModeSequenceTemplateMsg(ocs2::legged_robot::loadModeSequenceTemplate(file, "stance", false));
    trot_ = ocs2::legged_robot::createModeSequenceTemplateMsg(ocs2::legged_robot::loadModeSequenceTemplate(file, "trot", false));
    publisher_ = nh.advertise<ocs2_msgs::mode_schedule>("legged_robot_mpc_mode_schedule", 1, true);
    readySubscriber_ = nh.subscribe<std_msgs::Bool>("legged_robot_mpc_ready", 1,
                                                   [this](const std_msgs::Bool::ConstPtr& msg) { ready_ = msg->data; });
    service_ = privateNh.advertiseService("set_trot", &GaitCommand::setTrot, this);
    ROS_INFO("Gait service ready: set_trot(true) selects trot; false returns to stance.");
  }

 private:
  bool setTrot(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    if (request.data && !ready_) {
      response.message = "MPC/WBC is not ready; finish stand-up before requesting trot";
    } else if (publisher_.getNumSubscribers() == 0) {
      response.message = "No MPC gait receiver connected";
    } else {
      publisher_.publish(request.data ? trot_ : stance_);
      response.success = true;
      response.message = std::string(request.data ? "trot" : "stance") + " queued; applies after the current MPC horizon";
      ROS_INFO_STREAM(response.message);
    }
    return true;
  }

  bool ready_ = false;
  ocs2_msgs::mode_schedule stance_, trot_;
  ros::Publisher publisher_;
  ros::Subscriber readySubscriber_;
  ros::ServiceServer service_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "legged_gait_command");
  ros::NodeHandle nh, privateNh("~");
  try {
    GaitCommand command(nh, privateNh);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
