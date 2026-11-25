#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "capstone_integration/kinematics.hpp"

namespace capstone_integration {

// PLACEHOLDER for G1. Publishes the G1 sensor/localization contract from dead reckoning:
//   /sensors/scan (empty, "no obstacle"), /sensors/wheel_odom, /state/odom,
//   /localization/pose, /map and the TF chain map -> odom -> base_link.
// It exists so the rest of the system can be integrated before G1 delivers. It is not a
// localization algorithm: dead reckoning drifts, and the scan sees nothing.
class WorldStubNode : public rclcpp::Node {
 public:
  explicit WorldStubNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  Pose2D pose() const { return pose_; }
  double travelled_distance() const { return travelled_m_; }

 private:
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped &message);
  void publish_odometry();
  void publish_scan_and_pose();
  void publish_map();

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  Pose2D pose_;
  Pose2D start_pose_;
  double last_v_{0.0};
  double last_w_{0.0};
  double travelled_m_{0.0};
  double last_cmd_stamp_s_{0.0};
  bool have_cmd_stamp_{false};
  int map_publish_count_{0};
  bool sensors_disabled_{false};  // fault injection: sensor dropout
  bool publish_localization_{true};  // false: a real localization node owns sensors, odom and TF

  double map_resolution_{0.1};
  double map_width_m_{20.0};
  double map_height_m_{20.0};
  double map_origin_x_{-10.0};
  double map_origin_y_{-10.0};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dropout_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace capstone_integration
