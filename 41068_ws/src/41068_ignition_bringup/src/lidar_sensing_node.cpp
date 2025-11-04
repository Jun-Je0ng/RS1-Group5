#include "lidar_sensing_node.h"

LidarSensingNode::LidarSensingNode() : Node("lidar_sensing_node") {
    this->declare_parameter<double>("distance_threshold", 5.0);  // Increased to 5.0m
    this->declare_parameter<int>("angle_range", 60);             // Increased to 60 degrees
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    angle_range_deg_ = this->get_parameter("angle_range").as_int();

    pub_ = create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&LidarSensingNode::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LIDAR sensing node started with threshold %.2f m, angle range %d deg",
                distance_threshold_, angle_range_deg_);
}

LidarSensingNode::~LidarSensingNode() {
    RCLCPP_INFO(this->get_logger(), "LIDAR sensing node shutting down");
}

void LidarSensingNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double angle_range_rad = angle_range_deg_ * M_PI / 180.0;
    size_t num_rays = msg->ranges.size();
    size_t forward_idx = num_rays / 2;  // Approx forward direction (index 180 for 360 rays)
    size_t start_idx = forward_idx - static_cast<size_t>(angle_range_rad / msg->angle_increment);
    size_t end_idx = forward_idx + static_cast<size_t>(angle_range_rad / msg->angle_increment);

    // Clamp indices to valid range
    start_idx = std::max(static_cast<size_t>(0), start_idx);
    end_idx = std::min(num_rays - 1, end_idx);

    // Initialize OpenCV matrix for LiDAR data (e.g., 1D array or polar image)
    lidar_image_ = cv::Mat(num_rays, 1, CV_32F, cv::Scalar(0));
    for (size_t i = 0; i < num_rays; ++i) {
        lidar_image_.at<float>(i, 0) = (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) ? msg->ranges[i] : distance_threshold_ + 1;
    }

    // Apply OpenCV processing
    cv::Mat binary_image;
    cv::threshold(lidar_image_, binary_image, distance_threshold_, 255, cv::THRESH_BINARY_INV);
    cv::dilate(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), 2); // Remove noise
    cv::erode(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), 1);

    // Check for obstacles in the processed image
    bool obstacle_detected = false;
    double min_distance = distance_threshold_;  // Track the closest obstacle
    for (size_t i = start_idx; i <= end_idx; ++i) {
        if (binary_image.at<uchar>(i, 0) > 0) {
            obstacle_detected = true;
            min_distance = std::min(min_distance, static_cast<double>(msg->ranges[i]));  // Cast to double
        }
    }

    auto msg_out = std_msgs::msg::Bool();
    msg_out.data = obstacle_detected;
    pub_->publish(msg_out);

    if (obstacle_detected) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f meters", min_distance);
    } else {
        RCLCPP_INFO(this->get_logger(), "No obstacles detected within %.2f meters", distance_threshold_);
    }
}