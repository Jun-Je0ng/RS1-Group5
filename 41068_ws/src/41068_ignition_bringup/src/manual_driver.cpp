#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ManualDriver : public rclcpp::Node {
public:
  ManualDriver() : rclcpp::Node("manual_driver") {
    // Declare params (initial values can be overridden via --ros-args -p ...)
    topic_   = declare_parameter<std::string>("topic", "/manual/cmd_vel");
    lin_x_   = declare_parameter<double>("lin_x", 0.0);
    ang_z_   = declare_parameter<double>("ang_z", 0.0);
    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
    make_timer();  // build timer from rate_hz_

    // Dynamic parameter callback
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto &p : params) {
          if (p.get_name() == "lin_x" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            lin_x_ = p.as_double();
          } else if (p.get_name() == "ang_z" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            ang_z_ = p.as_double();
          } else if (p.get_name() == "rate_hz" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            rate_hz_ = p.as_double();
            make_timer();  // rebuild timer when rate changes
          } else if (p.get_name() == "topic" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            topic_ = p.as_string();
            pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
          }
        }
        RCLCPP_INFO(get_logger(), "Params -> lin_x=%.3f, ang_z=%.3f, rate=%.1f Hz, topic=%s",
                    lin_x_, ang_z_, rate_hz_, topic_.c_str());
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
      }
    );

    RCLCPP_INFO(get_logger(), "manual_driver publishing to %s (lin_x=%.2f, ang_z=%.2f, %.0f Hz)",
                topic_.c_str(), lin_x_, ang_z_, rate_hz_);
  }

private:
  void make_timer() {
    auto period_ms = static_cast<int>(1000.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ManualDriver::tick, this)
    );
  }

  void tick() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = lin_x_;
    msg.angular.z = ang_z_;
    pub_->publish(msg);
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // State/params
  std::string topic_;
  double lin_x_{0.2};
  double ang_z_{0.0};
  double rate_hz_{20.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualDriver>());
  rclcpp::shutdown();
  return 0;
}
