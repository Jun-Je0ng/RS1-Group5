#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/qos.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>


class AutonomousTrailFollower : public rclcpp::Node {
public:
 AutonomousTrailFollower()
 : Node("autonomous_trail_follower"),
   linear_vel_(declare_parameter("linear_velocity", 0.25)),
   angular_vel_(declare_parameter("angular_velocity", 1.0)),
   obstacle_detected_(false),
   trail_detected_(false),
   trail_centroid_x_(0.0),
   trail_centroid_y_(0.0),
   angular_gain_(declare_parameter("angular_gain", 0.002)),
   max_angular_speed_(declare_parameter("max_angular_speed", 1.5)),
   color_sample_window_px_(declare_parameter("color_sample_window_px", 40)),
   last_image_width_(0),
   has_color_sample_(false),
   last_trail_color_bgr_(0.0, 0.0, 0.0, 0.0) {

   const std::string image_topic =
       declare_parameter<std::string>("image_topic", "/camera/image");
   cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
   obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
       "/obstacle_detected", rclcpp::SystemDefaultsQoS(),
       std::bind(&AutonomousTrailFollower::obstacle_callback, this, std::placeholders::_1));
   image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
       image_topic, rclcpp::SensorDataQoS(),
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


     cv::Scalar lower(10, 50, 20);
     cv::Scalar upper(30, 255, 200);
     cv::Mat mask;
     cv::inRange(hsv, lower, upper, mask);


     cv::Moments m = cv::moments(mask, false);
     if (m.m00 > 1000) {
       trail_centroid_x_ = m.m10 / m.m00;
       trail_centroid_y_ = m.m01 / m.m00;
       trail_detected_ = true;
       last_image_width_ = cv_ptr->image.cols;
       update_trail_color_sample(cv_ptr->image, mask);
       RCLCPP_DEBUG(this->get_logger(), "Trail detected at x=%.1f (image width=%d)",
                    trail_centroid_x_, last_image_width_);
     } else {
       trail_detected_ = false;
       RCLCPP_DEBUG(this->get_logger(), "Trail lost");
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


 void timer_callback() {
   auto msg = geometry_msgs::msg::Twist();


   if (obstacle_detected_) {
     if (trail_detected_) {
       const double image_center = last_image_width_ > 0 ? last_image_width_ / 2.0 : 320.0;
       double error = trail_centroid_x_ - image_center;
       msg.angular.z = std::clamp(-error * angular_gain_, -max_angular_speed_, max_angular_speed_);
       msg.linear.x = 0.1;
     } else {
       msg.angular.z = 0.8;
     }
   } else if (trail_detected_) {
     const double image_center = last_image_width_ > 0 ? last_image_width_ / 2.0 : 320.0;
     double error = trail_centroid_x_ - image_center;
     msg.angular.z = std::clamp(-error * angular_gain_, -max_angular_speed_, max_angular_speed_);
     const double error_ratio = std::min(1.0, std::abs(error) / std::max(image_center, 1.0));
     msg.linear.x = std::clamp(linear_vel_ * (1.0 - error_ratio), 0.05, linear_vel_);
     RCLCPP_INFO_THROTTLE(
         this->get_logger(), *this->get_clock(), 2000,
         "Following trail: error=%.1f angular=%.2f linear=%.2f", error, msg.angular.z,
         msg.linear.x);
     if (has_color_sample_) {
       RCLCPP_INFO_THROTTLE(
           this->get_logger(), *this->get_clock(), 3000,
           "Trail colour sample (RGB): [%.0f, %.0f, %.0f]", last_trail_color_bgr_[2],
           last_trail_color_bgr_[1], last_trail_color_bgr_[0]);
     }
   } else {
     return;
   }


   cmd_vel_pub_->publish(msg);
 }


 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
 rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
 rclcpp::TimerBase::SharedPtr timer_;


 double linear_vel_, angular_vel_;
 bool obstacle_detected_, trail_detected_;
 double trail_centroid_x_, trail_centroid_y_;
 double angular_gain_;
 double max_angular_speed_;
 int color_sample_window_px_;
 int last_image_width_;
 bool has_color_sample_;
 cv::Scalar last_trail_color_bgr_;

 void update_trail_color_sample(const cv::Mat &image, const cv::Mat &mask) {
   if (color_sample_window_px_ <= 0) {
     has_color_sample_ = false;
     return;
   }

   const int half = color_sample_window_px_ / 2;
   const int cx = static_cast<int>(std::round(trail_centroid_x_));
   const int cy = static_cast<int>(std::round(trail_centroid_y_));

   const int x0 = std::max(0, cx - half);
   const int y0 = std::max(0, cy - half);
   const int x1 = std::min(image.cols, cx + half);
   const int y1 = std::min(image.rows, cy + half);

   if (x1 <= x0 || y1 <= y0) {
     has_color_sample_ = false;
     return;
   }

   const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
   cv::Mat image_roi = image(roi);
   cv::Mat mask_roi = mask(roi);

   const int sample_pixels = cv::countNonZero(mask_roi);
   if (sample_pixels == 0) {
     has_color_sample_ = false;
     return;
   }

   const cv::Scalar mean_bgr = cv::mean(image_roi, mask_roi);

   last_trail_color_bgr_ = mean_bgr;
   has_color_sample_ = true;
 }
};


int main(int argc, char **argv) {
 rclcpp::init(argc, argv);
 rclcpp::spin(std::make_shared<AutonomousTrailFollower>());
 rclcpp::shutdown();
 return 0;
}
