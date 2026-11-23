#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "capstone_integration/path_builder.hpp"

namespace capstone_integration {

// PLACEHOLDER for G4's FollowPath server. Pure-pursuit follower that publishes
// /control/cmd_vel_raw; the real safety monitor still filters everything through /cmd_vel.
class DriveStubNode : public rclcpp::Node {
 public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

  explicit DriveStubNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void tick();
  void on_odom(const nav_msgs::msg::Odometry &message);
  void publish_command(double v, double w);
  void finish(uint16_t code, const std::string &message);
  void reset_active();

  static std::vector<Pose2D> path_from_message(const nav_msgs::msg::Path &path);
  double seconds_since_start() const;

  FollowerParams params_;
  double position_tolerance_m_{0.25};
  double yaw_tolerance_rad_{0.2};
  double timeout_s_{180.0};
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  std::vector<Pose2D> path_;
  Pose2D pose_;
  bool has_pose_{false};
  std::optional<std::shared_ptr<GoalHandle>> active_;
  std::chrono::steady_clock::time_point started_{};
  int stop_ticks_{0};
  double last_v_{0.0};
  bool paused_{false};  // fault injection: stale command

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr crash_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp_action::Server<FollowPath>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace capstone_integration
