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
find_package(rcl_interfaces REQUIRED)

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
target_link_libraries(autonomous_trail_follower ${OpenCV_LIBS})
target_include_directories(autonomous_trail_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
add_executable(teleop_with_obstacle_avoidance src/teleop_with_obstacle_avoidance.cpp)
ament_target_dependencies(teleop_with_obstacle_avoidance
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
# Executable for MoveBot
add_executable(move_bot src/main_move_bot.cpp)  # Note: This might need adjustment if main.cpp is reused
target_link_libraries(move_bot controller)

# Waypoint and teleop helpers
add_executable(cmd_mux src/waypoint/cmd_mux.cpp)
ament_target_dependencies(cmd_mux rclcpp geometry_msgs std_msgs)

add_executable(manual_driver src/waypoint/manual_driver.cpp)
ament_target_dependencies(manual_driver rclcpp geometry_msgs rcl_interfaces)

add_executable(waypoint_manager src/waypoint/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs std_msgs)

# Install targets
install(TARGETS
  move_bot
  lidar_sensing_node
  autonomous_trail_follower
  teleop_with_obstacle_avoidance
  controller
  cmd_mux
  manual_driver
  waypoint_manager
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