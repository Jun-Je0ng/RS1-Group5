#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
struct Waypoint
{
  double x;
  double y;
  double yaw;
};

geometry_msgs::msg::PoseStamped makePose(
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
}  // namespace

class MapWaypointManager : public rclcpp::Node
{
public:
  MapWaypointManager()
  : rclcpp::Node("map_waypoint_manager"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters describing the mesh footprint and behaviour
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    waypoint_frame_ = declare_parameter<std::string>(
      "waypoint_frame", "map");
    transform_timeout_sec_ = declare_parameter<double>(
      "transform_timeout", 0.2);
    republish_rate_hz_ = declare_parameter<double>("republish_rate_hz", 1.0);
    auto_advance_ = declare_parameter<bool>("auto_advance", true);
    generate_perimeter_ = declare_parameter<bool>("generate_perimeter", false);
    manual_first_ = declare_parameter<bool>("manual_first", true);

    const double center_x = declare_parameter<double>("map_center_x", 0.0);
    const double center_y = declare_parameter<double>("map_center_y", 0.0);
    const double size_x = declare_parameter<double>("map_size_x", 50.0);
    const double size_y = declare_parameter<double>("map_size_y", 50.0);
    const double spacing = declare_parameter<double>("perimeter_spacing", 10.0);
    include_center_ = declare_parameter<bool>("include_center", false);
    manual_waypoints_ = declare_parameter<std::vector<double>>(
      "manual_waypoints",
      std::vector<double>{
        // Default patrol points (map frame): add/remove as needed
        20.793, 59.426, -1.472,
        29.696, 51.403, -1.038
      });

    if (generate_perimeter_) {
      generatePerimeterWaypoints(center_x, center_y, size_x, size_y, spacing);
    } else {
      RCLCPP_INFO(get_logger(), "Perimeter generation disabled via parameter.");
    }
    appendManualWaypoints();
    transformed_cache_.assign(waypoints_.size(), std::nullopt);

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("next_waypoint", latched_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", latched_qos);

    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "waypoint_reached", 10,
      std::bind(&MapWaypointManager::handleReached, this, std::placeholders::_1));

    select_sub_ = create_subscription<std_msgs::msg::Int32>(
      "select_waypoint", 10,
      std::bind(&MapWaypointManager::handleSelection, this, std::placeholders::_1));

    const auto period_ms =
      static_cast<int>(1000.0 / std::max(0.1, republish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&MapWaypointManager::tick, this));

    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(),
        "No waypoints were prepared. Check parameters or provide manual_waypoints.");
    } else {
      RCLCPP_INFO(get_logger(), "Prepared %zu waypoints (auto_advance=%s).",
        waypoints_.size(), auto_advance_ ? "true" : "false");
    }

    publishCurrent();
    publishMarkers();
  }

private:
  void generatePerimeterWaypoints(
    double center_x, double center_y,
    double size_x, double size_y,
    double spacing)
  {
    waypoints_.clear();
    if (size_x <= 0.0 || size_y <= 0.0 || spacing <= 0.0) {
      RCLCPP_ERROR(get_logger(),
        "Invalid parameters (size_x=%.3f, size_y=%.3f, spacing=%.3f).",
        size_x, size_y, spacing);
      return;
    }

    const double half_x = 0.5 * size_x;
    const double half_y = 0.5 * size_y;
    const double min_x = center_x - half_x;
    const double max_x = center_x + half_x;
    const double min_y = center_y - half_y;
    const double max_y = center_y + half_y;

    auto add_edge = [&](double x0, double y0, double x1, double y1, bool include_end) {
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double length = std::hypot(dx, dy);
      const int steps = std::max(1, static_cast<int>(std::floor(length / spacing)));
      const double yaw = std::atan2(dy, dx);

      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const double x = x0 + t * dx;
        const double y = y0 + t * dy;
        waypoints_.push_back({x, y, yaw});
      }

      if (include_end) {
        waypoints_.push_back({x1, y1, yaw});
      }
    };

    add_edge(min_x, min_y, max_x, min_y, false);
    add_edge(max_x, min_y, max_x, max_y, false);
    add_edge(max_x, max_y, min_x, max_y, false);
    add_edge(min_x, max_y, min_x, min_y, true);

    if (include_center_) {
      waypoints_.push_back({center_x, center_y, 0.0});
    }
  }

  void appendManualWaypoints()
  {
    if (manual_waypoints_.empty()) {
      return;
    }
    if (manual_waypoints_.size() % 3 != 0) {
      RCLCPP_WARN(
        get_logger(),
        "manual_waypoints must be a flat list of [x, y, yaw] triples. Ignoring trailing values.");
    }

    const std::size_t triples = manual_waypoints_.size() / 3;
    std::vector<Waypoint> manual;
    manual.reserve(triples);
    for (std::size_t i = 0; i < triples; ++i) {
      const double x = manual_waypoints_[3 * i + 0];
      const double y = manual_waypoints_[3 * i + 1];
      const double yaw = manual_waypoints_[3 * i + 2];
      manual.push_back({x, y, yaw});
    }

    if (manual_first_) {
      waypoints_.insert(waypoints_.begin(), manual.begin(), manual.end());
      for (std::size_t i = 0; i < manual.size(); ++i) {
        RCLCPP_INFO(get_logger(),
          "Added manual waypoint %zu -> (%.3f, %.3f, yaw=%.3f rad).",
          i, manual[i].x, manual[i].y, manual[i].yaw);
      }
    } else {
      const auto offset = waypoints_.size();
      waypoints_.insert(waypoints_.end(), manual.begin(), manual.end());
      for (std::size_t i = 0; i < manual.size(); ++i) {
        RCLCPP_INFO(get_logger(),
          "Added manual waypoint %zu -> (%.3f, %.3f, yaw=%.3f rad).",
          offset + i, manual[i].x, manual[i].y, manual[i].yaw);
      }
    }
  }

  std::optional<geometry_msgs::msg::PoseStamped> transformWaypoint(std::size_t index)
  {
    if (index >= waypoints_.size()) {
      return std::nullopt;
    }

    if (frame_id_ == waypoint_frame_) {
      return makePose(
        waypoints_[index].x, waypoints_[index].y, waypoints_[index].yaw,
        waypoint_frame_, now());
    }

    if (index < transformed_cache_.size() && transformed_cache_[index]) {
      transformed_cache_[index]->header.stamp = now();
      return transformed_cache_[index];
    }

    auto pose_in = makePose(
      waypoints_[index].x, waypoints_[index].y, waypoints_[index].yaw,
      waypoint_frame_, now());

    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame_id_, waypoint_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));

      geometry_msgs::msg::PoseStamped pose_out;
      tf2::doTransform(pose_in, pose_out, transform);

      if (index >= transformed_cache_.size()) {
        transformed_cache_.resize(waypoints_.size());
      }
      transformed_cache_[index] = pose_out;
      return transformed_cache_[index];
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform waypoint %zu from %s to %s: %s",
        index, waypoint_frame_.c_str(), frame_id_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void publishCurrent()
  {
    if (waypoints_.empty()) {
      return;
    }
    if (idx_ >= waypoints_.size()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "All perimeter waypoints completed.");
      return;
    }

    const auto &wp = waypoints_[idx_];
    auto pose_opt = transformWaypoint(idx_);
    if (!pose_opt) return;

    goal_pub_->publish(*pose_opt);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Publishing waypoint %zu at (%.2f, %.2f) yaw %.2f rad.",
      idx_, wp.x, wp.y, wp.yaw);
  }

  void publishMarkers()
  {
    if (waypoints_.empty()) {
      return;
    }

    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();

    visualization_msgs::msg::Marker spheres;
    spheres.header.frame_id = waypoint_frame_;
    spheres.header.stamp = stamp;
    spheres.ns = "waypoints";
    spheres.id = 0;
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::msg::Marker::ADD;
    spheres.scale.x = 0.6;
    spheres.scale.y = 0.6;
    spheres.scale.z = 0.2;
    spheres.color.a = 0.9f;
    spheres.color.g = 1.0f;

    spheres.points.reserve(waypoints_.size());
    for (const auto &wp : waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = wp.x;
      p.y = wp.y;
      p.z = 0.1;
      spheres.points.push_back(p);
    }
    array.markers.push_back(spheres);

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      visualization_msgs::msg::Marker label;
      label.header.frame_id = waypoint_frame_;
      label.header.stamp = stamp;
      label.ns = "waypoint_labels";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = waypoints_[i].x;
      label.pose.position.y = waypoints_[i].y;
      label.pose.position.z = 1.2;
      label.scale.z = 0.6;
      label.color.a = 1.0f;
      label.color.r = 1.0f;
      label.text = std::to_string(i);
      array.markers.push_back(label);
    }

    if (idx_ < waypoints_.size()) {
      visualization_msgs::msg::Marker arrow;
      auto current_pose = makePose(
        waypoints_[idx_].x, waypoints_[idx_].y, waypoints_[idx_].yaw,
        waypoint_frame_, stamp);

      arrow = visualization_msgs::msg::Marker();
      arrow.header.frame_id = waypoint_frame_;
      arrow.header.stamp = stamp;
      arrow.ns = "selected_waypoint";
      arrow.id = 0;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = current_pose.pose;
      arrow.scale.x = 2.0;
      arrow.scale.y = 0.3;
      arrow.scale.z = 0.3;
      arrow.color.a = 1.0f;
      arrow.color.r = 1.0f;
      arrow.color.g = 0.4f;
      arrow.color.b = 0.0f;
      array.markers.push_back(arrow);
    }

    marker_pub_->publish(array);
  }

  void handleReached(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || waypoints_.empty()) {
      return;
    }
    if (idx_ < waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "Robot reported waypoint %zu reached.", idx_);
      if (auto_advance_) {
        ++idx_;
        if (idx_ >= waypoints_.size()) {
          RCLCPP_INFO(get_logger(), "Perimeter patrol complete.");
        }
        publishCurrent();
        publishMarkers();
      }
    }
  }

  void handleSelection(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data < 0) {
      RCLCPP_WARN(get_logger(),
        "Waypoint index %d is negative; ignoring.", msg->data);
      return;
    }
    const auto requested = static_cast<std::size_t>(msg->data);
    if (requested >= waypoints_.size()) {
      RCLCPP_WARN(get_logger(),
        "Waypoint index %zu out of range (max %zu).",
        requested, waypoints_.empty() ? 0 : waypoints_.size() - 1);
      return;
    }

    idx_ = requested;
    RCLCPP_INFO(get_logger(), "Selected waypoint %zu manually.", idx_);
    publishCurrent();
    publishMarkers();
  }

  void tick()
  {
    publishCurrent();
    publishMarkers();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr select_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Waypoint> waypoints_;
  std::string frame_id_;
  std::string waypoint_frame_;
  double transform_timeout_sec_{0.2};
  double republish_rate_hz_{1.0};
  bool auto_advance_{true};
  bool include_center_{false};
  bool generate_perimeter_{true};
  bool manual_first_{false};
  std::vector<double> manual_waypoints_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::optional<geometry_msgs::msg::PoseStamped>> transformed_cache_;
  std::size_t idx_{0};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapWaypointManager>());
  rclcpp::shutdown();
  return 0;
}
