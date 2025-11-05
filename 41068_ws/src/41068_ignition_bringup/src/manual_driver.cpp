#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ManualDriver : public rclcpp::Node {
public:
  ManualDriver() : rclcpp::Node("manual_driver") {
    // Parameters (can be changed at runtime with ros2 param set)
    topic_     = declare_parameter<std::string>("topic", "/manual/cmd_vel");
    rate_hz_   = declare_parameter<double>("rate_hz", 20.0);

    // Linear velocities
    lin_x_     = declare_parameter<double>("lin_x", 0.0);  // forward/back
    lin_y_     = declare_parameter<double>("lin_y", 0.0);  // strafe L/R (drone)
    lin_z_     = declare_parameter<double>("lin_z", 0.0);  // up/down (drone)

    // Angular (yaw) velocity
    ang_z_     = declare_parameter<double>("ang_z", 0.0);  // yaw rate

    // Optional safety limits (clamp outputs)
    max_lin_   = declare_parameter<double>("max_lin", 3.0);   // m/s
    max_ang_   = declare_parameter<double>("max_ang", 2.0);   // rad/s

    pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
    make_timer();

    // Dynamic parameters
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto &p : params) {
          const auto &n = p.get_name();
          if (n == "topic" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            topic_ = p.as_string();
            pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
          } else if (n == "rate_hz") {
            rate_hz_ = p.as_double();
            make_timer();
          } else if (n == "lin_x") { lin_x_ = p.as_double(); }
          else if (n == "lin_y")    { lin_y_ = p.as_double(); }
          else if (n == "lin_z")    { lin_z_ = p.as_double(); }
          else if (n == "ang_z")    { ang_z_ = p.as_double(); }
          else if (n == "max_lin")  { max_lin_ = p.as_double(); }
          else if (n == "max_ang")  { max_ang_ = p.as_double(); }
        }
        RCLCPP_INFO(get_logger(),
          "manual_driver -> topic=%s  rate=%.1f Hz  lin[x,y,z]=[%.2f, %.2f, %.2f]  ang_z=%.2f  limits lin<=%.2f ang<=%.2f",
          topic_.c_str(), rate_hz_, lin_x_, lin_y_, lin_z_, ang_z_, max_lin_, max_ang_);
        rcl_interfaces::msg::SetParametersResult r; r.successful = true; r.reason = "ok"; return r;
      }
    );

    RCLCPP_INFO(get_logger(), "manual_driver publishing to %s (%.1f Hz)", topic_.c_str(), rate_hz_);
  }

private:
  void make_timer() {
    auto period_ms = static_cast<int>(1000.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&ManualDriver::tick, this));
  }

  template<typename T>
  T clamp(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

  void tick() {
    geometry_msgs::msg::Twist cmd;

    // Clamp for safety
    cmd.linear.x  = clamp(lin_x_, -max_lin_, max_lin_);
    cmd.linear.y  = clamp(lin_y_, -max_lin_, max_lin_);
    cmd.linear.z  = clamp(lin_z_, -max_lin_, max_lin_);
    cmd.angular.z = clamp(ang_z_, -max_ang_, max_ang_);

    pub_->publish(cmd);
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Params/state
  std::string topic_;
  double rate_hz_;
  double lin_x_, lin_y_, lin_z_;
  double ang_z_;
  double max_lin_, max_ang_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualDriver>());
  rclcpp::shutdown();
  return 0;
}
