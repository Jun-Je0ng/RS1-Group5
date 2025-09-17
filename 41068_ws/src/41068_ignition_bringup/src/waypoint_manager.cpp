#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

// Helper to make a PoseStamped with yaw (yaw is optional here)
static geometry_msgs::msg::PoseStamped makePose(double x, double y,
                                                const std::string &frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.w = 1.0; // identity quaternion
  return msg;
}

class WaypointManager : public rclcpp::Node {
public:
  WaypointManager() : rclcpp::Node("waypoint_manager") {
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    republish_rate_hz_ = declare_parameter<double>("republish_rate_hz", 1.0);

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("next_waypoint", 10);
    sub_ = create_subscription<std_msgs::msg::Bool>(
      "waypoint_reached", 10,
      std::bind(&WaypointManager::reachedCb, this, std::placeholders::_1));

    // TODO: replace with loading from a YAML/params file if desired.
    // For now, hardcode a few sample points (replace with your trail coordinates).
    waypoints_ = {
      { 17.0, 51.0 },   // example from your screenshot log
      { 25.0, 51.0 },
      { 32.0, 48.0 },
      { 40.0, 45.0 },
      { 48.0, 42.0 }
    };

    // Periodically re-publish the current waypoint (helps late joiners)
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(0.1, republish_rate_hz_)));
    timer_ = create_wall_timer(period, std::bind(&WaypointManager::publishCurrent, this));

    // Publish first waypoint immediately
    publishCurrent();
  }

private:
  struct WP { double x; double y; };

  void publishCurrent() {
    if (idx_ >= waypoints_.size()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "All waypoints completed.");
      return;
    }
    geometry_msgs::msg::PoseStamped msg = makePose(waypoints_[idx_].x, waypoints_[idx_].y, frame_id_);
    msg.header.stamp = now();
    pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Published waypoint %zu: (%.2f, %.2f)", idx_,
                         waypoints_[idx_].x, waypoints_[idx_].y);
  }

  void reachedCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    if (idx_ < waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu.", idx_);
      idx_++;
      if (idx_ < waypoints_.size()) {
        publishCurrent();
      } else {
        RCLCPP_INFO(get_logger(), "Mission complete. No more waypoints.");
      }
    }
  }

  // pubs/subs
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // config
  std::string frame_id_{"odom"};
  double republish_rate_hz_{1.0};

  // state
  std::vector<WP> waypoints_;
  std::size_t idx_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}
