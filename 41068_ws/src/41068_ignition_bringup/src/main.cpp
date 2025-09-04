#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "controller.h"

class MoveBot : public rclcpp::Node {
public:
    MoveBot() : Node("move_bot") {
        // Declare parameters
        this->declare_parameter<double>("lin_x", 0.0);  // Default forward velocity
        this->declare_parameter<double>("ang_z", 0.5);  // Default rotation speed

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        controller_ = std::make_shared<Controller>();

        // Timer to periodically send velocity commands
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&MoveBot::send_velocity, this));  // Call send_velocity periodically
    }

private:
    void send_velocity() {
        // Get parameters (e.g., lin_x, ang_z)
        double lin_x = this->get_parameter("lin_x").as_double();
        double ang_z = this->get_parameter("ang_z").as_double();

        geometry_msgs::msg::Twist msg;

        msg.linear.x = lin_x;   // Set the forward velocity from parameter
        msg.angular.z = ang_z;  // Set the angular velocity from parameter

        pub_->publish(msg);  // Publish to /cmd_vel

        // Send the same velocities to SkidSteer for control
        controller_->sendVelocityCommand(msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;  // Publisher for /cmd_vel
    std::shared_ptr<Controller> controller_;  // Controller object to manage movement
    rclcpp::TimerBase::SharedPtr timer_;  // Timer to call send_velocity periodically
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<MoveBot>());  // Spin the MoveBot node to keep it running
    rclcpp::shutdown();  // Shutdown ROS 2 when done
    return 0;
}
