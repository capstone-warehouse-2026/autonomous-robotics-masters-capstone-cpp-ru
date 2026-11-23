#include "capstone_integration/world_stub_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace capstone_integration {

namespace {

using namespace std::chrono_literals;

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double stamp_seconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

WorldStubNode::WorldStubNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("world_stub", options) {
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  start_pose_.x = declare_parameter<double>("start_x", -8.0);
  start_pose_.y = declare_parameter<double>("start_y", 0.0);
  start_pose_.yaw = declare_parameter<double>("start_yaw", 0.0);
  pose_ = start_pose_;

  const double odom_rate = declare_parameter<double>("odom_rate_hz", 50.0);
  const double scan_rate = declare_parameter<double>("scan_rate_hz", 10.0);
  map_resolution_ = declare_parameter<double>("map_resolution", 0.1);
  map_width_m_ = declare_parameter<double>("map_width_m", 20.0);
  map_height_m_ = declare_parameter<double>("map_height_m", 20.0);
  map_origin_x_ = declare_parameter<double>("map_origin_x", -10.0);
  map_origin_y_ = declare_parameter<double>("map_origin_y", -10.0);

  if (odom_rate <= 0.0 || scan_rate <= 0.0 || map_resolution_ <= 0.0 || map_width_m_ <= 0.0 ||
      map_height_m_ <= 0.0) {
    throw std::invalid_argument("world_stub: rates and map dimensions must be positive");
  }

  cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::TwistStamped &message) { on_cmd_vel(message); });

  wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sensors/wheel_odom", 10);
  state_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/state/odom", rclcpp::QoS(5).reliable());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", rclcpp::QoS(5).reliable());
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/sensors/scan", rclcpp::SensorDataQoS());
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = now();
  map_to_odom.header.frame_id = map_frame_;
  map_to_odom.child_frame_id = odom_frame_;
  map_to_odom.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(map_to_odom);

  const auto odom_period = std::chrono::duration<double>(1.0 / odom_rate);
  const auto scan_period = std::chrono::duration<double>(1.0 / scan_rate);
  odom_timer_ = create_wall_timer(odom_period, [this]() { publish_odometry(); });
  scan_timer_ = create_wall_timer(scan_period, [this]() { publish_scan_and_pose(); });
  // Three attempts: /map uses transient_local, but a late subscriber still needs the sample.
  map_timer_ = create_wall_timer(500ms, [this]() {
    publish_map();
    if (++map_publish_count_ >= 3) {
      map_timer_->cancel();
    }
  });

  RCLCPP_WARN(get_logger(),
              "PLACEHOLDER G1: dead-reckoning sensors and localization from /cmd_vel; "
              "empty scan; map is free space. Replace with capstone_localization.");
}

void WorldStubNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped &message) {
  const double stamp = stamp_seconds(message.header.stamp);
  double dt = 0.05;
  if (have_cmd_stamp_) {
    dt = stamp - last_cmd_stamp_s_;
    if (!(dt > 0.0)) {
      dt = 1.0 / 20.0;
    }
    dt = clamp(dt, 0.0, 0.5);
  }

  const double v = std::isfinite(message.twist.linear.x) ? message.twist.linear.x : 0.0;
  const double w = std::isfinite(message.twist.angular.z) ? message.twist.angular.z : 0.0;
  pose_ = integrate_twist(pose_, v, w, dt);
  travelled_m_ += std::abs(v) * dt;
  last_v_ = v;
  last_w_ = w;
  last_cmd_stamp_s_ = stamp;
  have_cmd_stamp_ = true;
}

void WorldStubNode::publish_odometry() {
  const rclcpp::Time stamp = now();
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;
  odometry.pose.pose.position.x = pose_.x;
  odometry.pose.pose.position.y = pose_.y;
  odometry.pose.pose.orientation = yaw_to_quaternion(pose_.yaw);
  odometry.pose.covariance = {0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.05};
  odometry.twist.twist.linear.x = last_v_;
  odometry.twist.twist.angular.z = last_w_;
  odometry.twist.covariance = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.02};
  wheel_odom_pub_->publish(odometry);
  state_odom_pub_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = pose_.x;
  transform.transform.translation.y = pose_.y;
  transform.transform.rotation = yaw_to_quaternion(pose_.yaw);
  tf_broadcaster_->sendTransform(transform);
}

void WorldStubNode::publish_scan_and_pose() {
  const rclcpp::Time stamp = now();

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = base_frame_;
  scan.angle_min = static_cast<float>(-kPi);
  scan.angle_max = static_cast<float>(kPi);
  scan.angle_increment = static_cast<float>(2.0 * kPi / 360.0);
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.assign(360, std::numeric_limits<float>::infinity());
  scan_pub_->publish(scan);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.pose.position.x = pose_.x;
  pose.pose.pose.position.y = pose_.y;
  pose.pose.pose.orientation = yaw_to_quaternion(pose_.yaw);
  pose.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02};
  pose_pub_->publish(pose);
}

void WorldStubNode::publish_map() {
  const auto width = static_cast<std::uint32_t>(std::lround(map_width_m_ / map_resolution_));
  const auto height = static_cast<std::uint32_t>(std::lround(map_height_m_ / map_resolution_));

  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = now();
  map.header.frame_id = map_frame_;
  map.info.resolution = static_cast<float>(map_resolution_);
  map.info.width = width;
  map.info.height = height;
  map.info.origin.position.x = map_origin_x_;
  map.info.origin.position.y = map_origin_y_;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(static_cast<std::size_t>(width) * height, 0);

  for (std::uint32_t x = 0; x < width; ++x) {
    map.data[x] = 100;
    map.data[(height - 1) * width + x] = 100;
  }
  for (std::uint32_t y = 0; y < height; ++y) {
    map.data[y * width] = 100;
    map.data[y * width + (width - 1)] = 100;
  }
  map_pub_->publish(map);
}

}  // namespace capstone_integration
