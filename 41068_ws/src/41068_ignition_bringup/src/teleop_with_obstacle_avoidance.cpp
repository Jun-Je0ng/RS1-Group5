#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
    
            // Laser scan subscriber for reactive avoidance
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TeleopObstacleAvoid::scan_callback, this, std::placeholders::_1));

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

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // parameters
    const double threshold = 0.8;           // meters, distance considered "obstacle"
    const double frontal_deg = 30.0;        // sector half-width in degrees
    const int n = static_cast<int>(msg->ranges.size());
    const double angle_min = msg->angle_min;
    const double angle_inc = msg->angle_increment;

    double min_range = std::numeric_limits<double>::infinity();
    double min_angle = 0.0;

    // examine frontal window around 0 radians
    for (int i = 0; i < n; ++i) {
      double angle = angle_min + i * angle_inc;
      double angle_deg = angle * 180.0 / M_PI;
      if (std::abs(angle_deg) > frontal_deg) continue;
      double r = msg->ranges[i];
      if (std::isfinite(r) && r < min_range) {
        min_range = r;
        min_angle = angle;  // radians, negative = left, positive = right depending on topics
      }
    }

    if (min_range < threshold) {
      obstacle_detected_ = true;
      avoidance_mode_ = true;
      // decide turn direction: if obstacle is left (angle < 0) turn right (+1), else left (-1)
      avoid_turn_dir_ = (min_angle < 0.0) ? 1.0 : -1.0;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Reactive avoid: range=%.2f m angle=%.1fdeg dir=%+.0f",
                           min_range, min_angle*180.0/M_PI, avoid_turn_dir_);
    } else {
      // clear if no close obstacles in frontal sector
      if (avoidance_mode_) {
        // small hysteresis: require clear for a short time -> here immediate clear
        avoidance_mode_ = false;
      }
      // external obstacle_detected_ (std_msgs) still honored
      obstacle_detected_ = obstacle_detected_; // no-op keep existing flag
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
        // If reactive avoidance active, override teleop and drive around
    if (avoidance_mode_) {
      auto avoid = geometry_msgs::msg::Twist();
      // turn while moving slowly forward to arc around obstacle
      avoid.linear.x = linear_vel_ * 0.4;
      avoid.angular.z = avoid_turn_dir_ * angular_vel_ * 0.8;
      cmd_vel_pub_->publish(avoid);
      return;
    }

    // If external obstacle flag set (e.g. camera), stop and do not accept teleop
    if (obstacle_detected_) {
      publish_zero_velocity();
      return;
    }

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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double linear_vel_;
  double angular_vel_;
  bool obstacle_detected_;

  bool avoidance_mode_;
  double avoid_turn_dir_;
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