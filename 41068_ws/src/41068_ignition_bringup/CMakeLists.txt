cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Library for controller and skidsteer
add_library(controller src/controller.cpp src/skidsteer.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs)

# Executable for Lidar Sensing Node
add_executable(lidar_sensing_node
  src/lidar_sensing_node.cpp
  src/main.cpp  # Use the main.cpp specific to lidar_sensing_node
)
ament_target_dependencies(lidar_sensing_node rclcpp sensor_msgs std_msgs)
target_include_directories(lidar_sensing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
add_executable(autonomous_trail_follower src/autnomous_trail_follower.cpp)
ament_target_dependencies(autonomous_trail_follower
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)
add_executable(teleop_with_obstacle_avoidance src/teleop_with_obstacle_avoidance.cpp)
ament_target_dependencies(teleop_with_obstacle_avoidance
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
target_link_libraries(autonomous_trail_follower ${OpenCV_INCLUDE_DIRS})
target_include_directories(autonomous_trail_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Executable for MoveBot
add_executable(move_bot src/main_move_bot.cpp)  # Note: This might need adjustment if main.cpp is reused
target_link_libraries(move_bot controller)

# Install targets
install(TARGETS move_bot lidar_sensing_node autonomous_trail_follower teleop_with_obstacle_avoidance controller
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Install package resources ----
install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# Lint/tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()