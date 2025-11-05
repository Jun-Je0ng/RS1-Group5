cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

# C++ standard (optional but recommended)
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
# ros_ign_interfaces was in your old file; keep only if you still use it elsewhere
# find_package(ros_ign_interfaces REQUIRED)

# -------------------- Executables ---------------------
# waypoint_follower replaces your old main.cpp
add_executable(waypoint_follower src/main.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# waypoint_manager is the new publisher of sequential waypoints
add_executable(waypoint_manager src/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager
  rclcpp
  geometry_msgs
  std_msgs
)

# -------------------- Install -------------------------
install(TARGETS
  waypoint_follower
  waypoint_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Package resources (unchanged)
install(
  DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# -------------------- Lint / Tests --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the RViz click-to-YAML helper so it can be run with ros2 run
install(PROGRAMS
  scripts/click_to_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()