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

  /* -------------------------------------------------------------
     Non‑blocking single‑character keyboard input
     ------------------------------------------------------------- */
  char getch() {
    char buf = 0;
    struct termios old{};
    tcgetattr(0, &old);
    struct termios raw = old;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &raw);
    read(0, &buf, 1);
    tcsetattr(0, TCSANOW, &old);
    return buf;
  }

  void timer_callback() {
    if (obstacle_detected_) return;   // safety stop already sent

    char key = getch();
    auto msg = geometry_msgs::msg::Twist();

    // Handle normal letters
    switch (key) {
      case 'w': case 'W':
        msg.linear.x = linear_vel_;
        break;
      case 's': case 'S':
        msg.linear.x = -linear_vel_ * 0.5;
        break;
      case 'a': case 'A':
        msg.angular.z = angular_vel_;
        break;
      case 'd': case 'D':
        msg.angular.z = -angular_vel_;
        break;
      case ' ':
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        break;
      default:
        // Not a letter → check for arrow keys
        if (key == 27 && read(0, &key, 1) == 1 && key == '[') {  // ESC [ sequence
          if (read(0, &key, 1) == 1) {
            switch (key) {
              case 'A':  // Up
                msg.linear.x = linear_vel_;
                break;
              case 'B':  // Down
                msg.linear.x = -linear_vel_ * 0.5;
                break;
              case 'D':  // Left
                msg.angular.z = angular_vel_;
                break;
              case 'C':  // Right
                msg.angular.z = -angular_vel_;
                break;
            }
          }
        }
        break;
    }

    if (msg.linear.x != 0.0 || msg.angular.z != 0.0) {
      cmd_vel_pub_->publish(msg);
    }
  }

  // Members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double linear_vel_;
  double angular_vel_;
  bool obstacle_detected_;
};

/* -------------------------------------------------------------
   MAIN
   ------------------------------------------------------------- */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopObstacleAvoid>());
  rclcpp::shutdown();
  return 0;
}