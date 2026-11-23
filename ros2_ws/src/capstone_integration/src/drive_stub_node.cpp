#include "capstone_integration/drive_stub_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace capstone_integration {

namespace {

using namespace std::chrono_literals;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

DriveStubNode::DriveStubNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("drive_stub", options) {
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");

  params_.lookahead_m = declare_parameter<double>("lookahead_m", 0.8);
  params_.max_v = declare_parameter<double>("max_v", 0.5);
  params_.max_w = declare_parameter<double>("max_w", 1.0);
  params_.heading_gain = declare_parameter<double>("heading_gain", 1.5);
  params_.turn_in_place_angle_rad = declare_parameter<double>("turn_in_place_angle_rad", 0.6);
  params_.arrive_radius_m = declare_parameter<double>("arrive_radius_m", 0.15);
  position_tolerance_m_ = declare_parameter<double>("goal_position_tolerance_m", 0.25);
  yaw_tolerance_rad_ = declare_parameter<double>("goal_yaw_tolerance_rad", 0.2);
  timeout_s_ = declare_parameter<double>("timeout_s", 180.0);

  if (!(params_.max_v > 0.0) || !(params_.max_w > 0.0) || !(timeout_s_ > 0.0) ||
      !(position_tolerance_m_ > 0.0) || !(yaw_tolerance_rad_ > 0.0)) {
    throw std::invalid_argument("drive_stub: limits and tolerances must be positive");
  }

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/control/cmd_vel_raw", rclcpp::QoS(1).reliable());
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state/odom", rclcpp::QoS(5).reliable(),
      [this](const nav_msgs::msg::Odometry &message) { on_odom(message); });

  server_ = rclcpp_action::create_server<FollowPath>(
      this, "/control/follow_path",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const FollowPath::Goal> goal) { return handle_goal(uuid, goal); },
      [this](const std::shared_ptr<GoalHandle> goal_handle) { return handle_cancel(goal_handle); },
      [this](const std::shared_ptr<GoalHandle> goal_handle) { handle_accepted(goal_handle); });

  timer_ = create_wall_timer(50ms, [this]() { tick(); });

  RCLCPP_WARN(get_logger(),
              "PLACEHOLDER G4: pure-pursuit follower on /control/follow_path publishing "
              "/control/cmd_vel_raw. Replace with the group control node.");
}

void DriveStubNode::on_odom(const nav_msgs::msg::Odometry &message) {
  pose_ = Pose2D{message.pose.pose.position.x, message.pose.pose.position.y,
                 yaw_from_quaternion(message.pose.pose.orientation)};
  has_pose_ = true;
}

rclcpp_action::GoalResponse DriveStubNode::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal) {
  if (goal->path.poses.empty()) {
    RCLCPP_WARN(get_logger(), "rejecting goal with an empty path");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DriveStubNode::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DriveStubNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
  path_ = path_from_message(goal_handle->get_goal()->path);
  active_ = goal_handle;
  started_ = std::chrono::steady_clock::now();
  stop_ticks_ = 0;
  last_v_ = 0.0;
  RCLCPP_INFO(get_logger(), "new FollowPath goal: %zu poses", path_.size());
}

double DriveStubNode::seconds_since_start() const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
}

std::vector<Pose2D> DriveStubNode::path_from_message(const nav_msgs::msg::Path &path) {
  std::vector<Pose2D> points;
  points.reserve(path.poses.size());
  for (const auto &pose : path.poses) {
    const auto &p = pose.pose;
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y)) {
      return {};
    }
    points.push_back(Pose2D{p.position.x, p.position.y, yaw_from_quaternion(p.orientation)});
  }
  return points;
}

void DriveStubNode::publish_command(double v, double w) {
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = base_frame_;
  command.twist.linear.x = v;
  command.twist.angular.z = w;
  cmd_pub_->publish(command);
  last_v_ = v;
}

void DriveStubNode::tick() {
  if (!active_.has_value()) {
    return;
  }
  const auto goal_handle = active_.value();

  if (goal_handle->is_canceling()) {
    publish_command(0.0, 0.0);
    auto result = std::make_shared<FollowPath::Result>();
    result->error_code = FollowPath::Result::UNKNOWN;
    result->error_msg = "cancelled";
    goal_handle->canceled(result);
    reset_active();
    return;
  }

  if (seconds_since_start() > timeout_s_) {
    finish(FollowPath::Result::CONTROLLER_TIMED_OUT, "placeholder follower timed out");
    return;
  }
  if (path_.empty()) {
    finish(FollowPath::Result::INVALID_PATH, "empty or non-finite path");
    return;
  }
  if (!has_pose_) {
    publish_command(0.0, 0.0);
    if (seconds_since_start() > 3.0) {
      finish(FollowPath::Result::NO_VALID_CONTROL, "no odometry on /state/odom");
    }
    return;
  }

  const Pose2D &goal = path_.back();
  auto feedback = std::make_shared<FollowPath::Feedback>();
  feedback->distance_to_goal = static_cast<float>(remaining_distance(path_, pose_));

  if (distance(pose_, goal) <= position_tolerance_m_) {
    const double yaw_error = angle_diff(goal.yaw, pose_.yaw);
    if (std::abs(yaw_error) <= yaw_tolerance_rad_) {
      publish_command(0.0, 0.0);
      feedback->speed = 0.0F;
      goal_handle->publish_feedback(feedback);
      if (++stop_ticks_ >= 4) {
        finish(FollowPath::Result::NONE, "goal reached");
      }
      return;
    }
    stop_ticks_ = 0;
    const double w = clamp(params_.heading_gain * yaw_error, -params_.max_w, params_.max_w);
    publish_command(0.0, w);
    feedback->speed = 0.0F;
    goal_handle->publish_feedback(feedback);
    return;
  }

  stop_ticks_ = 0;
  const auto command = follow_path(path_, pose_, params_);
  publish_command(command.v, command.w);
  feedback->speed = static_cast<float>(command.v);
  goal_handle->publish_feedback(feedback);
}

void DriveStubNode::finish(uint16_t code, const std::string &message) {
  if (!active_.has_value()) {
    return;
  }
  const auto goal_handle = active_.value();
  auto result = std::make_shared<FollowPath::Result>();
  result->error_code = code;
  result->error_msg = message;
  if (code == FollowPath::Result::NONE) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  reset_active();
}

void DriveStubNode::reset_active() {
  active_.reset();
  path_.clear();
  stop_ticks_ = 0;
}

}  // namespace capstone_integration
