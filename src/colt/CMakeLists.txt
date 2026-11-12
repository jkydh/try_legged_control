cmake_minimum_required(VERSION 2.8.3)

project(colt)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/bringup.py scripts/trot_forward.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

find_package(roslaunch)

foreach(dir config launch meshes urdf worlds)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
