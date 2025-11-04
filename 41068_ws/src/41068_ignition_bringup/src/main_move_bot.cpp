#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "controller.h"
#include "std_msgs/msg/bool.hpp"

class MoveBot : public rclcpp::Node {
public:
    MoveBot() : Node("move_bot") {
        this->declare_parameter<double>("lin_x", 0.0);
        this->declare_parameter<double>("ang_z", 0.5);

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        controller_ = std::make_shared<Controller>();

        sub_ = create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected",
            10,
            std::bind(&MoveBot::obstacle_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MoveBot::send_velocity, this));
    }

private:
    void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        obstacle_detected_ = msg->data;
    }

    void send_velocity() {
        double lin_x = this->get_parameter("lin_x").as_double();
        double ang_z = this->get_parameter("ang_z").as_double();

        geometry_msgs::msg::Twist msg;
        if (obstacle_detected_) {
            lin_x = 0.0;  // Stop forward movement
            ang_z = 0.5;  // Turn to avoid
        }
        msg.linear.x = lin_x;
        msg.angular.z = ang_z;

        pub_->publish(msg);
        controller_->sendVelocityCommand(msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::shared_ptr<Controller> controller_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    bool obstacle_detected_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveBot>());
    rclcpp::shutdown();
    return 0;
}