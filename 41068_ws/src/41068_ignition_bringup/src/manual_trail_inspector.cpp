#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

class ManualTrailInspector : public rclcpp::Node {
public:
  ManualTrailInspector()
  : Node("manual_trail_inspector"),
    color_sample_window_px_(declare_parameter<int>("color_sample_window_px", 40)),
    min_trail_area_(declare_parameter<int>("min_trail_area", 1000)),
    trail_centroid_x_(0.0),
    trail_centroid_y_(0.0) {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image");

    const std::vector<double> lower_default{10.0, 50.0, 20.0};
    const std::vector<double> upper_default{30.0, 255.0, 200.0};
    auto lower = declare_parameter<std::vector<double>>("lower_hsv", lower_default);
    auto upper = declare_parameter<std::vector<double>>("upper_hsv", upper_default);

    if (lower.size() != 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter 'lower_hsv' must have exactly 3 elements. Using defaults.");
      lower = lower_default;
    }
    if (upper.size() != 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter 'upper_hsv' must have exactly 3 elements. Using defaults.");
      upper = upper_default;
    }

    lower_hsv_ = cv::Scalar(lower[0], lower[1], lower[2]);
    upper_hsv_ = cv::Scalar(upper[0], upper[1], upper[2]);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ManualTrailInspector::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Manual trail inspector ready. Subscribing to %s (HSV lower=[%.0f, %.0f, %.0f], "
                "upper=[%.0f, %.0f, %.0f])",
                image_topic_.c_str(), lower_hsv_[0], lower_hsv_[1], lower_hsv_[2], upper_hsv_[0],
                upper_hsv_[1], upper_hsv_[2]);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat hsv;
      cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

      cv::Mat mask;
      cv::inRange(hsv, lower_hsv_, upper_hsv_, mask);

      const cv::Moments m = cv::moments(mask, false);
      if (m.m00 < static_cast<double>(min_trail_area_)) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Trail not detected (area %.0f < %d)", m.m00, min_trail_area_);
        return;
      }

      trail_centroid_x_ = m.m10 / m.m00;
      trail_centroid_y_ = m.m01 / m.m00;

      if (update_color_sample(cv_ptr->image, mask)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Trail colour sample (RGB): [%.0f, %.0f, %.0f] at (%.0f, %.0f)",
                             last_trail_color_bgr_[2], last_trail_color_bgr_[1],
                             last_trail_color_bgr_[0], trail_centroid_x_, trail_centroid_y_);
      }
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  bool update_color_sample(const cv::Mat &image, const cv::Mat &mask) {
    if (color_sample_window_px_ <= 0) {
      return false;
    }

    const int half = color_sample_window_px_ / 2;
    const int cx = static_cast<int>(std::lround(trail_centroid_x_));
    const int cy = static_cast<int>(std::lround(trail_centroid_y_));

    const int x0 = std::clamp(cx - half, 0, image.cols);
    const int y0 = std::clamp(cy - half, 0, image.rows);
    const int x1 = std::clamp(cx + half, 0, image.cols);
    const int y1 = std::clamp(cy + half, 0, image.rows);

    if (x1 <= x0 || y1 <= y0) {
      return false;
    }

    const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
    const cv::Mat image_roi = image(roi);
    const cv::Mat mask_roi = mask(roi);

    const int sample_pixels = cv::countNonZero(mask_roi);
    if (sample_pixels == 0) {
      return false;
    }

    last_trail_color_bgr_ = cv::mean(image_roi, mask_roi);
    return true;
  }

  std::string image_topic_;
  cv::Scalar lower_hsv_;
  cv::Scalar upper_hsv_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  int color_sample_window_px_;
  int min_trail_area_;
  double trail_centroid_x_;
  double trail_centroid_y_;
  cv::Scalar last_trail_color_bgr_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualTrailInspector>());
  rclcpp::shutdown();
  return 0;
}