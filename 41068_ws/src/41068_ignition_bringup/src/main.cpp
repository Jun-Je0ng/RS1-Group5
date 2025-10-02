#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

static inline double wrapToPi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

static inline double yawFromQuat(double x, double y, double z, double w) {
  // yaw (Z) from quaternion
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

class WaypointFollower : public rclcpp::Node {
public:
  WaypointFollower() : rclcpp::Node("waypoint_follower") {
    // Control gains & limits
    k_lin_ = declare_parameter("k_lin", 0.8);           // linear gain
    k_ang_ = declare_parameter("k_ang", 2.5);           // angular gain
    v_max_ = declare_parameter("v_max", 0.8);           // max linear speed (m/s)
    w_max_ = declare_parameter("w_max", 1.2);           // max angular speed (rad/s)
    tol_dist_ = declare_parameter("tol_dist", 0.25);    // distance threshold (m)
    tol_yaw_  = declare_parameter("tol_yaw", 0.25);     // (optional) final yaw tol (rad)
    align_angle_ = declare_parameter("align_angle", 0.6); // slow forward if |err_yaw| > this

    // Topic names (configurable)
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");   // <- was "/odom"
    goal_topic_ = declare_parameter<std::string>("goal_topic", "next_waypoint");
    cmd_topic_  = declare_parameter<std::string>("cmd_topic", "/autonomy/cmd_vel");     // <- use mux output

    // Pubs/Subs
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("waypoint_reached", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&WaypointFollower::odomCb, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&WaypointFollower::goalCb, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&WaypointFollower::controlLoop, this)); // 20 Hz

    RCLCPP_INFO(get_logger(), "Follower started. Topics -> odom: %s, goal: %s, cmd: %s",
                odom_topic_.c_str(), goal_topic_.c_str(), cmd_topic_.c_str());
  }

private:
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_ = *msg;
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "Received goal: (%.3f, %.3f)",
                goal_.pose.position.x, goal_.pose.position.y);
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_ = *msg;
    have_odom_ = true;
  }

  void controlLoop() {
    if (!have_goal_) {
        auto clk = this->get_clock();
        RCLCPP_INFO_THROTTLE(get_logger(), *clk, 2000, "Waiting for next_waypoint...");
        return;
    }
    if (!have_odom_) {
        auto clk = this->get_clock();
        RCLCPP_INFO_THROTTLE(get_logger(), *clk, 2000,
                            "Waiting for odom on '%s'...", odom_topic_.c_str());
        return;
    }

    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);

    const double gx = goal_.pose.position.x;
    const double gy = goal_.pose.position.y;

    const double dx = gx - p.x;
    const double dy = gy - p.y;

    const double dist = std::hypot(dx, dy);
    const double heading = std::atan2(dy, dx);
    const double err_yaw = wrapToPi(heading - yaw);

    geometry_msgs::msg::Twist cmd;

    if (dist < tol_dist_) {
      // stop
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);

      std_msgs::msg::Bool flag; flag.data = true;
      reached_pub_->publish(flag);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waypoint reached.");
      return;
    }

    // P controller
    double v = k_lin_ * dist;
    double w = k_ang_ * err_yaw;

    // Reduce forward speed while turning significantly
    if (std::abs(err_yaw) > align_angle_) v *= 0.2;

    cmd.linear.x  = clamp(v, 0.0, v_max_);
    cmd.angular.z = clamp(w, -w_max_, w_max_);

    cmd_pub_->publish(cmd);
  }

  // Publishers/Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::PoseStamped goal_;
  bool have_odom_{false};
  bool have_goal_{false};

  // Params
  double k_lin_{0.8}, k_ang_{2.5}, v_max_{0.8}, w_max_{1.2};
  double tol_dist_{0.25}, tol_yaw_{0.25}, align_angle_{0.6};
  std::string odom_topic_, goal_topic_, cmd_topic_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollower>());
  rclcpp::shutdown();
  return 0;
}
