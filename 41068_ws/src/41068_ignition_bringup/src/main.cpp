#include "lidar_sensing_node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSensingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}