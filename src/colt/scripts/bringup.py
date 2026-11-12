#!/usr/bin/env python3
"""Spawn in crouch, load controllers, unpause and wait for a manual start."""
import threading
import time

import rospy
from controller_manager_msgs.srv import LoadController, SwitchController, SwitchControllerRequest
from gazebo_msgs.srv import GetPhysicsProperties, SetModelConfiguration, SpawnModel
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


def service(name, service_type):
    rospy.wait_for_service(name, timeout=120.0)
    return rospy.ServiceProxy(name, service_type)


def main():
    rospy.init_node("colt_bringup")
    physics = service("/gazebo/get_physics_properties", GetPhysicsProperties)()
    if not physics.success or not physics.pause:
        raise RuntimeError("Colt bringup requires Gazebo physics to be paused")
    pose = Pose()
    pose.position.z = rospy.get_param("~base_height")
    pose.orientation.w = 1.0
    result = service("/gazebo/spawn_urdf_model", SpawnModel)(
        "colt", rospy.get_param("/robot_description"), "/", pose, "world")
    if not result.success:
        raise RuntimeError(result.status_message)
    result = service("/gazebo/set_model_configuration", SetModelConfiguration)(
        "colt", "robot_description", rospy.get_param("~joint_names"), rospy.get_param("~joint_positions"))
    if not result.success:
        raise RuntimeError(result.status_message)

    # Gazebo's p3d plugin does not publish while paused. Seed the known initial
    # pose so the cheater estimator has valid data on its first control update.
    initial_odom = Odometry()
    initial_odom.header.frame_id = "world"
    initial_odom.child_frame_id = "base"
    initial_odom.pose.pose = pose
    seed_publisher = rospy.Publisher("/ground_truth/state", Odometry, queue_size=1, latch=True)
    seed_publisher.publish(initial_odom)

    controller = "controllers/legged_cheater_controller" if rospy.get_param("~cheater") else "controllers/legged_controller"
    load = service("/controller_manager/load_controller", LoadController)
    for name in ("controllers/joint_state_controller", controller):
        rospy.loginfo("Loading %s (CppAD compilation may take several minutes)", name)
        if not load(name).ok:
            raise RuntimeError("Failed to load " + name)
    switch = service("/controller_manager/switch_controller", SwitchController)
    unpause = service("/gazebo/unpause_physics", Empty)
    request = SwitchControllerRequest()
    request.start_controllers = ["controllers/joint_state_controller"]
    if rospy.get_param("~start_control"):
        request.start_controllers.append(controller)
    request.strictness = SwitchControllerRequest.STRICT
    # Switching is applied on the next physics update; another thread unpauses.
    result_holder = {}

    def activate():
        try:
            result_holder["response"] = switch(request)
        except Exception as error:
            result_holder["error"] = error

    worker = threading.Thread(target=activate, daemon=True)
    worker.start()
    time.sleep(0.1)  # Wall time: ROS simulation time is paused.
    unpause()
    worker.join(timeout=300.0)
    response = result_holder.get("response")
    if worker.is_alive() or response is None or not response.ok:
        service("/gazebo/pause_physics", Empty)()
        raise RuntimeError("Controller activation failed or timed out: " + str(result_holder.get("error", "no successful response")))
    if rospy.get_param("~start_control"):
        rospy.loginfo("Colt stand-up requested with %s; wait for MPC/WBC handover", controller)
    else:
        rospy.loginfo("Colt ready and unpowered; start %s via /controller_manager/switch_controller to stand up", controller)


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal("Colt bringup failed: %s", error)
        raise SystemExit(1)
