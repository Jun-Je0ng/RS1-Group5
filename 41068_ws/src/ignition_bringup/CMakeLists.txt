find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(autonomous_trail_follower
  src/Autonomous_trail_follower.cpp)   # be sure the filename matches EXACT case
ament_target_dependencies(autonomous_trail_follower
  rclcpp geometry_msgs std_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(autonomous_trail_follower ${OpenCV_LIBS})

add_executable(teleop_with_obstacle_avoidance
  src/teleop_with_obstacle_avoidance.cpp)
ament_target_dependencies(teleop_with_obstacle_avoidance
  rclcpp geometry_msgs std_msgs)

install(TARGETS
  autonomous_trail_follower
  teleop_with_obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME})

ament_package()
