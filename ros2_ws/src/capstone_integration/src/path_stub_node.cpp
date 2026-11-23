#include "capstone_integration/path_stub_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "capstone_integration/path_builder.hpp"

namespace capstone_integration {

namespace {

using namespace std::chrono_literals;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool finite_pose(const geometry_msgs::msg::Pose &pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

}  // namespace

PathStubNode::PathStubNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("path_stub", options) {
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  step_m_ = declare_parameter<double>("step_m", 0.1);
  half_extent_m_ = declare_parameter<double>("map_half_extent_m", 10.0);
  if (!(step_m_ > 0.0) || !(half_extent_m_ > 0.0)) {
    throw std::invalid_argument("path_stub: step_m and map_half_extent_m must be positive");
  }

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", rclcpp::QoS(5).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
        on_pose(message);
      });

  server_ = rclcpp_action::create_server<ComputePath>(
      this, "/planning/compute_path",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const ComputePath::Goal> goal) { return handle_goal(uuid, goal); },
      [this](const std::shared_ptr<GoalHandle> goal_handle) { return handle_cancel(goal_handle); },
      [this](const std::shared_ptr<GoalHandle> goal_handle) { handle_accepted(goal_handle); });

  timer_ = create_wall_timer(20ms, [this]() { execute_pending(); });

  RCLCPP_WARN(get_logger(),
              "PLACEHOLDER G3: straight-line planner on /planning/compute_path; "
              "obstacles are ignored. Replace with capstone_planning.");
}

void PathStubNode::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
  current_pose_ = Pose2D{message.pose.pose.position.x, message.pose.pose.position.y,
                         yaw_from_quaternion(message.pose.pose.orientation)};
  has_pose_ = true;
}

rclcpp_action::GoalResponse PathStubNode::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePath::Goal> goal) {
  if (!finite_pose(goal->goal.pose) || (goal->use_start && !finite_pose(goal->start.pose))) {
    RCLCPP_WARN(get_logger(), "rejecting goal with non-finite pose");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathStubNode::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathStubNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
  pending_ = goal_handle;
}

void PathStubNode::execute_pending() {
  if (!pending_.has_value()) {
    return;
  }
  const auto goal_handle = pending_.value();
  pending_.reset();

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<ComputePath::Result>();
    result->error_code = ComputePath::Result::UNKNOWN;
    result->error_msg = "cancelled";
    goal_handle->canceled(result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  Pose2D start;
  if (goal->use_start) {
    start = Pose2D{goal->start.pose.position.x, goal->start.pose.position.y,
                   yaw_from_quaternion(goal->start.pose.orientation)};
  } else if (has_pose_) {
    start = current_pose_;
  } else {
    auto result = std::make_shared<ComputePath::Result>();
    result->error_code = ComputePath::Result::TF_ERROR;
    result->error_msg = "no robot pose on /localization/pose yet";
    goal_handle->abort(result);
    return;
  }

  const Pose2D goal_pose{goal->goal.pose.position.x, goal->goal.pose.position.y,
                         yaw_from_quaternion(goal->goal.pose.orientation)};

  auto result = std::make_shared<ComputePath::Result>();
  if (std::abs(start.x) > half_extent_m_ || std::abs(start.y) > half_extent_m_) {
    result->error_code = ComputePath::Result::START_OUTSIDE_MAP;
    result->error_msg = "start outside the placeholder map";
    goal_handle->abort(result);
    return;
  }
  if (std::abs(goal_pose.x) > half_extent_m_ || std::abs(goal_pose.y) > half_extent_m_) {
    result->error_code = ComputePath::Result::GOAL_OUTSIDE_MAP;
    result->error_msg = "goal outside the placeholder map";
    goal_handle->abort(result);
    return;
  }

  const auto points = straight_path(start, goal_pose, step_m_);
  result->path.header.stamp = now();
  result->path.header.frame_id = goal->goal.header.frame_id.empty() ? map_frame_
                                                                   : goal->goal.header.frame_id;
  result->path.poses.reserve(points.size());
  const auto stamp = now();
  for (const auto &point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = result->path.header.frame_id;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.z = std::sin(point.yaw * 0.5);
    pose.pose.orientation.w = std::cos(point.yaw * 0.5);
    result->path.poses.push_back(pose);
  }
  result->planning_time.sec = 0;
  result->planning_time.nanosec = 1000000;
  result->error_code = ComputePath::Result::NONE;
  result->error_msg.clear();
  goal_handle->succeed(result);
}

}  // namespace capstone_integration
