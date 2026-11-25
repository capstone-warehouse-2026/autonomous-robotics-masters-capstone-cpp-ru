#pragma once

#include <chrono>
#include <optional>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "capstone_nav2/scan_grid.hpp"

namespace capstone_nav2 {

// Twist callback -> stamped raw command; scan callback -> grid; AMCL callback -> pose.
// Старые команды и измерения не переиздаются по таймеру с новым stamp.
class ContractsAdapter : public rclcpp::Node {
 public:
  explicit ContractsAdapter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
 private:
  using Clock = std::chrono::steady_clock;
  void read_parameters();
  void create_ros_interfaces();
  void on_command(const geometry_msgs::msg::Twist &command);
  void on_scan(const sensor_msgs::msg::LaserScan &scan);
  void process_pending_scan();
  void on_localization(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);
  void process_pending_localization();
  void request_localization_update();
  void on_clock(const rosgraph_msgs::msg::Clock &clock);
  bool restore_initial_pose();
  void publish_diagnostic(unsigned char level, const std::string &reason);

  GridParameters grid_parameters_;
  std::string base_frame_, grid_frame_;
  double max_input_age_{0.3};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr update_client_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr scan_retry_timer_;
  std::optional<int64_t> update_request_;
  Clock::time_point update_requested_;
  std::string last_diagnostic_;
  std::optional<sensor_msgs::msg::LaserScan> pending_scan_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_pose_;
  Clock::time_point scan_received_, pose_received_;
  double initial_x_{-3.0}, initial_y_{0.0}, initial_yaw_{0.0};
  double last_clock_{0.0};
  bool initial_pose_pending_{false};
  bool reset_confirmation_pending_{false};
  double initial_pose_stamp_{-1.0};
  unsigned request_generation_{0};
};

}  // namespace capstone_nav2
