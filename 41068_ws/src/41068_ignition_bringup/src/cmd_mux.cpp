#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

class CmdMux : public rclcpp::Node {
public:
  CmdMux() : rclcpp::Node("cmd_mux") {
    // Parameters
    out_topic_   = declare_parameter<std::string>("out_topic", "/cmd_vel");
    auto_topic_  = declare_parameter<std::string>("autonomy_topic", "/autonomy/cmd_vel");
    man_topic_   = declare_parameter<std::string>("manual_topic", "/manual/cmd_vel");
    enabled_     = declare_parameter<bool>("autonomy_enabled", true);

    pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);

    sub_auto_ = create_subscription<geometry_msgs::msg::Twist>(
      auto_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        last_auto_ = *msg;
        if (enabled_) pub_->publish(*msg);
      });

    sub_man_ = create_subscription<geometry_msgs::msg::Twist>(
      man_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        last_man_ = *msg;
        if (!enabled_) pub_->publish(*msg);
      });

    // Toggle autonomy on/off
    sub_toggle_ = create_subscription<std_msgs::msg::Bool>(
      "autonomy_enabled", 10, [this](std_msgs::msg::Bool::SharedPtr msg){
        enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "Autonomy %s", enabled_ ? "ENABLED" : "DISABLED");
        // Immediately publish the currently selected source so robot responds fast
        pub_->publish(enabled_ ? last_auto_ : last_man_);
      });

    RCLCPP_INFO(get_logger(), "cmd_mux up. Output: %s, auto: %s, manual: %s, autonomy_enabled=%s",
                out_topic_.c_str(), auto_topic_.c_str(), man_topic_.c_str(),
                enabled_ ? "true" : "false");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_auto_, sub_man_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_toggle_;
  geometry_msgs::msg::Twist last_auto_{}, last_man_{};
  std::string out_topic_, auto_topic_, man_topic_;
  bool enabled_{true};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdMux>());
  rclcpp::shutdown();
  return 0;
}
