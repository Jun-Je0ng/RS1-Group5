cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ---------- ROS dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


# ---------- OpenCV (system) ----------
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)


# ---------- controller library ----------
add_library(controller
 src/controller.cpp
 src/skidsteer.cpp
)
ament_target_dependencies(controller
 rclcpp
 geometry_msgs
)
target_include_directories(controller PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
)




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
 src/main.cpp
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


# ---------- move_bot ----------
add_executable(move_bot
 src/main_move_bot.cpp
)
ament_target_dependencies(move_bot
 rclcpp
 geometry_msgs
 std_msgs
)
target_link_libraries(move_bot controller)


# ---------- install all executables ----------
install(TARGETS
 lidar_sensing_node
 move_bot
 teleop_obstacle_avoid
 autonomous_trail_follower   # ← CRITICAL LINE
 manual_trail_inspector
 controller
 DESTINATION lib/${PROJECT_NAME}
)


install(
 DIRECTORY config launch models urdf worlds urdf_drone
 DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()


ament_package()
