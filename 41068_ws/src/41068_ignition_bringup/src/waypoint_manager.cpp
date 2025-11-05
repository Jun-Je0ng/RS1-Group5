#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

struct WP {
  double x;
  double y;
  double yaw;
};

// helper: planar yaw → quaternion
static geometry_msgs::msg::PoseStamped makePose(
    double x, double y, double yaw,
    const std::string &frame_id,
    const rclcpp::Time &stamp)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;

  const double half = 0.5 * yaw;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(half);
  msg.pose.orientation.w = std::cos(half);
  return msg;
}

class WaypointManager : public rclcpp::Node
{
public:
  WaypointManager() : rclcpp::Node("waypoint_manager")
  {
    // Parameters
    frame_id_          = declare_parameter<std::string>("frame_id", "odom");
    republish_rate_hz_ = declare_parameter<double>("republish_rate_hz", 1.0);

    // -------- Waypoints: (x, y, yaw_radians) --------
    waypoints_ = {
      {15, 50}
      // you can add more here like:
      // , {x2, y2, yaw2}
    };
    // ------------------------------------------------

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("next_waypoint", 10);
    sub_ = create_subscription<std_msgs::msg::Bool>(
      "waypoint_reached", 10,
      std::bind(&WaypointManager::reachedCb, this, std::placeholders::_1));

    const auto period_ms =
        static_cast<int>(1000.0 / std::max(0.1, republish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&WaypointManager::publishCurrent, this));

    publishCurrent(); // publish immediately
  }

private:
  void publishCurrent()
  {
    if (idx_ >= waypoints_.size()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                           "All waypoints completed.");
      return;
    }
    const auto &wp = waypoints_[idx_];
    auto msg = makePose(wp.x, wp.y, wp.yaw, frame_id_, now());
    pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Published waypoint %zu: (%.3f, %.3f, yaw=%.3f rad)",
      idx_, wp.x, wp.y, wp.yaw);
  }

  void reachedCb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;
    if (idx_ < waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu.", idx_);
      ++idx_;
      if (idx_ < waypoints_.size()) publishCurrent();
      else RCLCPP_INFO(get_logger(), "Mission complete. No more waypoints.");
    }
  }

  // ---- members ----
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_{"odom"};
  double republish_rate_hz_{1.0};
  std::vector<WP> waypoints_;
  std::size_t idx_{0};
};

// ---- main ----
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}
