cmake_minimum_required(VERSION 3.10)
project(h30_uart LANGUAGES CXX)
find_package(Threads REQUIRED)

# Build just the existing Linux driver, without catkin/ROS/OCS2/Gazebo.
set(H30_HW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src/legged_control/legged_hw")
add_library(h30_serial STATIC "${H30_HW_DIR}/src/H30Imu.cpp")
target_include_directories(h30_serial PUBLIC "${H30_HW_DIR}/include")
target_compile_features(h30_serial PUBLIC cxx_std_14)
target_compile_options(h30_serial PRIVATE -Wall -Wextra -Wpedantic)
add_executable(h30_read h30_read.cpp)
target_link_libraries(h30_read PRIVATE h30_serial Threads::Threads)
target_compile_options(h30_read PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS h30_read RUNTIME DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME h30_uart_pty
    COMMAND "${PYTHON_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test_h30_read.py" "$<TARGET_FILE:h30_read>")
endif()
