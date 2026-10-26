Colt (10) - RViz ready
=======================
This package is made from the user's colt(10).zip.
The original urdf/colt.urdf and all meshes/*.STL bytes are unchanged.

Modified:
 - launch/display.launch: remove required empty arg and fix RViz config path
 - CMakeLists.txt: install rviz directory
 - config/joint_names_colt.yaml: remove empty joint-name entry
Added:
 - rviz/colt.rviz: RobotModel, Fixed Frame=baselink, joint TF display disabled
 - RVIZ_README.txt: these instructions

ROS Noetic / catkin:
  cd ~/Desktop/try_legged_control
  source /opt/ros/noetic/setup.bash
  catkin build colt
  source devel/setup.bash
  roslaunch colt display.launch

Install the colt directory in ~/Desktop/try_legged_control/src/colt.
Back up existing colt first. If catkin is missing, ensure catkin_tools is installed.
RViz is a visualization tool, NOT a dynamics or collision verification tool.

Outstanding engineering questions:
 - All 12 revolute joint limits are +/-3.14 rad (check physical hardware limits).
 - Latest joint origin rpy settings differ from colt(9); test +/-0.1 rad joint movement.
 - urdf/colt.csv mass values differ from urdf/colt.urdf values; review SolidWorks exporter.
 - Four FOOT collision geometries use their original STL files.
 - gazebo.launch (unchanged) has a base_link vs baselink mismatch; not used by RViz.
