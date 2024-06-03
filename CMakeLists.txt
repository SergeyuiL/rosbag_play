cmake_minimum_required(VERSION 3.5)
project(my_rosbag_playback)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(playback_node src/playback_node.cpp)
ament_target_dependencies(playback_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_msgs
  geometry_msgs
  rosbag2_cpp
)

install(TARGETS
  playback_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
