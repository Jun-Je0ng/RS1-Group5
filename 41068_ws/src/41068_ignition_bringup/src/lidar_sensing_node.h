#ifndef LIDAR_SENSING_NODE_HPP
#define LIDAR_SENSING_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

class LidarSensingNode : public rclcpp::Node {
public:
    LidarSensingNode();
    ~LidarSensingNode();

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

    double distance_threshold_;
    int angle_range_deg_;
};

#endif // LIDAR_SENSING_NODE_HPP   