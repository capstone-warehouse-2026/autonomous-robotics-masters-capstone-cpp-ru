#pragma once

#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "capstone_integration/kinematics.hpp"

namespace capstone_integration {

// PLACEHOLDER for G3. Serves /planning/compute_path with a straight line. It ignores the obstacle
// grid on purpose: it proves the action contract, not the planning algorithm.
class PathStubNode : public rclcpp::Node {
 public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePath>;

  explicit PathStubNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const ComputePath::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute_pending();

  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message);

  std::string map_frame_;
  double step_m_{0.1};
  double half_extent_m_{10.0};
  Pose2D current_pose_;
  bool has_pose_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp_action::Server<ComputePath>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<std::shared_ptr<GoalHandle>> pending_;
};

}  // namespace capstone_integration
