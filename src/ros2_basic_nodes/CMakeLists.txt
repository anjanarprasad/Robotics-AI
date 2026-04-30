cmake_minimum_required(VERSION 3.8)
project(my_cpp20_pubsub)

# Use C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Dependencies
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Simulation monitor
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# TF / Odom / PoseArray bridge
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# PX4 ROS 2 messages
find_package(px4_msgs REQUIRED)

# ----------------------------
# Executables
# ----------------------------

# PX4 Offboard control node
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control
  rclcpp
  px4_msgs
)

# Basic pub/sub
add_executable(my_talker src/my_talker.cpp)
ament_target_dependencies(my_talker
  rclcpp
  std_msgs
)

add_executable(my_listener src/my_listener.cpp)
ament_target_dependencies(my_listener
  rclcpp
  std_msgs
)

# Simulation monitor
add_executable(sim_monitor src/sim_monitor.cpp)
ament_target_dependencies(sim_monitor
  rclcpp
  sensor_msgs
  rosgraph_msgs
)

# GT PoseArray -> TF/Odom broadcaster
add_executable(gt_posearray_tf src/gt_posearray_tf.cpp)
ament_target_dependencies(gt_posearray_tf
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ----------------------------
# Install
# ----------------------------
install(TARGETS
  offboard_control
  my_talker
  my_listener
  sim_monitor
  gt_posearray_tf
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()