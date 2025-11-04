#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <termios.h>
#include <unistd.h>
#include <chrono>

class TeleopObstacleAvoid : public rclcpp::Node {
public:
  TeleopObstacleAvoid()
  : Node("teleop_obstacle_avoid"), linear_vel_(0.3), angular_vel_(0.8), obstacle_detected_(false) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/obstacle_detected", 10,
        std::bind(&TeleopObstacleAvoid::obstacle_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&TeleopObstacleAvoid::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "Teleop with obstacle avoidance READY. Use WASD or arrows.");
    RCLCPP_INFO(this->get_logger(),
                "W/↑: Forward  |  S/↓: Backward  |  A/←: Turn Left  |  D/→: Turn Right  |  Space: STOP");
  }

private:
  void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    obstacle_detected_ = msg->data;
    if (obstacle_detected_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "OBSTACLE AHEAD — STOPPING HUSKY!");
      publish_zero_velocity();
    }
  }

  void publish_zero_velocity() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
  }

  char getch() {
    char buf = 0;
    struct termios old = {0};
    tcgetattr(0, &old);
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &old);
    read(0, &buf, 1);
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    tcsetattr(0, TCSANOW, &old);
    return buf;
  }

  void timer_callback() {
    if (obstacle_detected_) return;

    char key = getch();
    auto msg = geometry_msgs::msg::Twist();

    switch (key) {
      case 'w': case 'W': case 65:  // Up arrow
        msg.linear.x = linear_vel_;
        break;
      case 's': case 'S': case 66:  // Down arrow
        msg.linear.x = -linear_vel_ * 0.5;
        break;
      case 'a': case 'A': case 68:  // Left arrow
        msg.angular.z = angular_vel_;
        break;
      case 'd': case 'D': case 67:  // Right arrow
        msg.angular.z = -angular_vel_;
        break;
      case ' ':  // Space = emergency stop
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        break;
      default:
        return;
    }
    cmd_vel_pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double linear_vel_;
  double angular_vel_;
  bool obstacle_detected_;
};

/* -------------------------------------------------------------
   MAIN – THIS WAS MISSING
   ------------------------------------------------------------- */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopObstacleAvoid>());
  rclcpp::shutdown();
  return 0;
}