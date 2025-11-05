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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
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

add_executable(map_waypoint_manager src/map_waypoint_manager.cpp)
ament_target_dependencies(map_waypoint_manager
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------- OpenCV (system) ----------
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# ---------- teleop_obstacle_avoid ----------
add_executable(teleop_obstacle_avoid
 src/teleop_with_obstacle_avoidance.cpp
)
ament_target_dependencies(teleop_obstacle_avoid
 rclcpp
 geometry_msgs
 std_msgs
)




# ---------- autonomous_trail_follower ----------
add_executable(autonomous_trail_follower
 src/autonomous_trail_follower.cpp
)
ament_target_dependencies(autonomous_trail_follower
 rclcpp
 geometry_msgs
 std_msgs
 sensor_msgs
 cv_bridge   # ← NOW VALID
)
target_include_directories(autonomous_trail_follower PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(autonomous_trail_follower
 ${OpenCV_LIBRARIES}
 -lopencv_core
 -lopencv_imgproc
)
install(TARGETS autonomous_trail_follower
 DESTINATION lib/${PROJECT_NAME}
)


# ---------- manual_trail_inspector ----------
add_executable(manual_trail_inspector
 src/manual_trail_inspector.cpp
)
ament_target_dependencies(manual_trail_inspector
 rclcpp
 sensor_msgs
 cv_bridge
)
target_include_directories(manual_trail_inspector PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(manual_trail_inspector
 ${OpenCV_LIBRARIES}
 -lopencv_core
 -lopencv_imgproc
)


# ---------- lidar_sensing_node ----------
add_executable(lidar_sensing_node
 src/lidar_sensing_node.cpp
)
ament_target_dependencies(lidar_sensing_node
 rclcpp
 sensor_msgs
 std_msgs
)
target_include_directories(lidar_sensing_node PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lidar_sensing_node
 ${OpenCV_LIBRARIES}
 -lopencv_core
 -lopencv_imgproc
)


# -------------------- Install -------------------------
install(TARGETS
  waypoint_follower
  waypoint_manager
  map_waypoint_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Package resources (unchanged)
install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# -------------------- Lint / Tests --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the RViz click-to-YAML helper so it can be run with ros2 run
install(PROGRAMS
  scripts/waypoint_setter.py
  scripts/click_to_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
