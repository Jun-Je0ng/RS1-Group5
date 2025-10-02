cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

# -------------------- Build setup --------------------
# (helps VS Code find includes)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------- Dependencies --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Keep these only if you actually use Ignition-related bridges/launches elsewhere:
# find_package(ros_ign_gazebo REQUIRED)
# find_package(ros_ign_bridge REQUIRED)
# find_package(ros_ign_interfaces REQUIRED)

# -------------------- Executables ---------------------
# Waypoint follower (src/main.cpp)
add_executable(waypoint_follower src/main.cpp)
ament_target_dependencies(waypoint_follower rclcpp geometry_msgs nav_msgs std_msgs)

# Waypoint manager
add_executable(waypoint_manager src/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs std_msgs)

# Command multiplexer (selects manual vs autonomy)
add_executable(cmd_mux src/cmd_mux.cpp)
ament_target_dependencies(cmd_mux rclcpp geometry_msgs std_msgs)

# Manual driver (publishes constant lin_x / ang_z)
add_executable(manual_driver src/manual_driver.cpp)
ament_target_dependencies(manual_driver rclcpp geometry_msgs)

# -------------------- Install -------------------------
install(TARGETS
  waypoint_follower
  waypoint_manager
  cmd_mux
  manual_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Package resources
install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# -------------------- Lint / Tests --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
