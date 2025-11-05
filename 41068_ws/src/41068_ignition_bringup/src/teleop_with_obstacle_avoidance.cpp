#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("teleop_obstacle_avoid");
  RCLCPP_WARN(logger, "teleop_with_obstacle_avoidance node is not implemented yet.");
  rclcpp::shutdown();
  return 0;
}
