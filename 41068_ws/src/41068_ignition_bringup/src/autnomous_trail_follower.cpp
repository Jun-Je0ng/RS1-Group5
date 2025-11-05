#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>

class AutonomousTrailFollower : public rclcpp::Node {
public:
  AutonomousTrailFollower()
  : Node("autonomous_trail_follower"), linear_vel_(0.25), angular_vel_(1.0),
    obstacle_detected_(false), trail_detected_(false),
    trail_centroid_x_(0.0), trail_centroid_y_(0.0) {   // initialize centroids

    // Publishers & Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/obstacle_detected", 10,
        std::bind(&AutonomousTrailFollower::obstacle_callback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&AutonomousTrailFollower::image_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&AutonomousTrailFollower::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "AUTONOMOUS TRAIL FOLLOWER READY. Brown trail = guide. Obstacle = stop & avoid.");
  }

private:
  void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    obstacle_detected_ = msg->data;
    if (obstacle_detected_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "OBSTACLE AHEAD — STOPPING & AVOIDING!");
      publish_stop();
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat hsv;
      cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

      // Brown trail in HSV (tuned for typical forest trail)
      cv::Scalar lower_brown(10, 50, 20);
      cv::Scalar upper_brown(30, 255, 200);
      cv::Mat mask;
      cv::inRange(hsv, lower_brown, upper_brown, mask);

      // Find moments
      cv::Moments m = cv::moments(mask, false);
      if (m.m00 > 1000) {  // enough brown pixels
        trail_centroid_x_ = m.m10 / m.m00;
        trail_centroid_y_ = m.m01 / m.m00;
        trail_detected_ = true;
      } else {
        trail_detected_ = false;
      }
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void publish_stop() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
  }

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
    auto msg = geometry_msgs::msg::Twist();

    if (obstacle_detected_) {
      // If obstacle is detected, stop forward motion and turn in place to avoid.
      msg.linear.x = 0.0;
      msg.angular.z = angular_vel_;
    } else if (trail_detected_) {
      // Simple proportional controller to center on trail centroid.
      // NOTE: this uses a nominal image center (320). Adjust if camera resolution differs.
      const double image_center_x = 320.0;
      double error = trail_centroid_x_ - image_center_x;
      msg.linear.x = linear_vel_;
      msg.angular.z = -0.002 * error;  // gain tuned small to avoid oscillation
    } else {
      // No trail and no obstacle -> stop
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(msg);
  }

  // Members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_vel_, angular_vel_;
  bool obstacle_detected_, trail_detected_;
  double trail_centroid_x_, trail_centroid_y_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousTrailFollower>());
  rclcpp::shutdown();
  return 0;
}