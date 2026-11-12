cmake_minimum_required(VERSION 3.10)
project(legged_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        controller_manager
        urdf
        sensor_msgs
        tf2
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        h30_imu
        CATKIN_DEPENDS
        roscpp
        legged_common
        controller_manager
        urdf
        sensor_msgs
        tf2
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/LeggedHW.cpp
        src/LeggedHWLoop.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_library(h30_imu src/H30Imu.cpp)
target_compile_features(h30_imu PUBLIC cxx_std_14)
add_executable(h30_imu_node src/H30ImuNode.cpp)
target_compile_features(h30_imu_node PRIVATE cxx_std_14)
target_link_libraries(h30_imu_node h30_imu ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_install_python(PROGRAMS test/h30_ros_test.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_rostest(test/h30_ros.test DEPENDENCIES h30_imu_node)
    catkin_add_gtest(h30_imu_test test/H30ImuTest.cpp)
    if(TARGET h30_imu_test)
        target_link_libraries(h30_imu_test h30_imu gtest_main util)
        target_compile_features(h30_imu_test PRIVATE cxx_std_14)
    endif()
endif()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} h30_imu h30_imu_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
