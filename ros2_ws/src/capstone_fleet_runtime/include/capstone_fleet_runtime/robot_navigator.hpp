#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "capstone_fleet_runtime/timed_input.hpp"

namespace capstone_fleet_runtime {

class RobotNavigator {
 public:
  enum class ActionState { Idle, Pending, Active, Cancelling, Succeeded, Cancelled, Failed };
  RobotNavigator(rclcpp::Node &node, std::string robot_id, double timeout = 30.0);
  void update();
  bool ready() const;
  bool inputs_fresh() const;
  void follow_edge(double start_x, double start_y, double goal_x, double goal_y);
  void wait_at(double x, double y);
  void cancel();
  bool terminal() const { return terminal_received_ && !response_pending_; }
  bool stopped() const;
  bool stop_complete() const;
  bool settled_at(double x, double y) const;
  bool edge_complete() const;
  ActionState state() const { return state_; }
  const std::string &reason() const { return reason_; }
  double x() const;
  double y() const;
  double distance_remaining() const { return distance_remaining_; }

 private:
  using Clock = std::chrono::steady_clock;
  using Follow = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Follow>;
  void on_odometry(const nav_msgs::msg::Odometry &message);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message);
  void on_goal_response(std::uint64_t generation, const GoalHandle::SharedPtr &handle);
  void on_result(std::uint64_t generation, const GoalHandle::WrappedResult &result);
  void cancel_owned_goal();
  void update_stopped_interval();
  bool stopped_after(double stamp, Clock::time_point received) const;
  double distance_from_reserved_edge() const;
  static bool valid_stamp(const builtin_interfaces::msg::Time &stamp);

  rclcpp::Node &node_;
  std::string robot_id_, reason_;
  TimedInput<nav_msgs::msg::Odometry> odometry_;
  TimedInput<geometry_msgs::msg::PoseWithCovarianceStamped> pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp_action::Client<Follow>::SharedPtr client_;
  GoalHandle::SharedPtr handle_;
  ActionState state_{ActionState::Idle};
  bool response_pending_{false}, terminal_received_{true}, cancel_requested_{false};
  bool cancel_sent_{false}, reservation_active_{false};
  std::uint64_t generation_{0};
  double start_x_{0}, start_y_{0}, goal_x_{0}, goal_y_{0};
  double timeout_, result_stamp_{0}, stop_requested_stamp_{0}, distance_remaining_{0};
  double last_odom_stamp_{-1}, stopped_since_{-1};
  Clock::time_point goal_started_{}, result_received_{};
  std::optional<Clock::time_point> stopped_since_wall_, last_odom_received_, stop_requested_wall_;
};
}  // namespace capstone_fleet_runtime
