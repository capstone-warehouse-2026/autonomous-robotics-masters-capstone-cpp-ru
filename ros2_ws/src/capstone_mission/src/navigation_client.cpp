#include "capstone_mission/navigation_client.hpp"

#include <cmath>
#include <stdexcept>

namespace capstone_mission {
namespace {
bool valid_quaternion(const geometry_msgs::msg::Quaternion &q) {
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
    return false;
  }
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::abs(norm - 1.0) < 0.01;
}
double elapsed(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

NavigationClient::NavigationClient(rclcpp::Node &node, Limits limits, Event event)
    : node_(node), limits_(limits), event_(std::move(event)) {
  for (double value : {limits.planning_timeout, limits.navigation_timeout,
                       limits.stop_timeout, limits.input_timeout}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("navigation timeouts must be finite and positive");
    }
  }
  planner_ = rclcpp_action::create_client<Plan>(&node_, "planning/compute_path");
  controller_ = rclcpp_action::create_client<Follow>(&node_, "control/follow_path");
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      "state/odom", rclcpp::QoS(5).reliable(),
      [this](const nav_msgs::msg::Odometry &message) { on_odometry(message); });
}

bool NavigationClient::fresh_odometry() const {
  if (!odom_received_ || elapsed(odom_received_at_) > limits_.input_timeout) return false;
  const double age = (node_.now() - rclcpp::Time(odometry_.header.stamp)).seconds();
  return age >= -0.01 && age <= limits_.input_timeout;
}

bool NavigationClient::stopped() const {
  return fresh_odometry() && std::abs(odometry_.twist.twist.linear.x) < 0.05 &&
         std::abs(odometry_.twist.twist.angular.z) < 0.1;
}

bool NavigationClient::ready() const {
  return planner_->action_server_is_ready() && controller_->action_server_is_ready() &&
         fresh_odometry();
}

void NavigationClient::on_odometry(const nav_msgs::msg::Odometry &message) {
  const auto &position = message.pose.pose.position;
  const auto &orientation = message.pose.pose.orientation;
  if (message.header.frame_id != "odom" || message.child_frame_id != "base_link" ||
      !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !valid_quaternion(orientation) ||
      !std::isfinite(message.twist.twist.linear.x) ||
      !std::isfinite(message.twist.twist.angular.z)) return;
  odometry_ = message;
  odom_received_at_ = Clock::now();
  odom_received_ = true;
}

void NavigationClient::start(const geometry_msgs::msg::PoseStamped &target) {
  if (state_ == State::Planning || state_ == State::Following || state_ == State::Stopping ||
      (state_ == State::Failed && !stop_confirmed_)) {
    throw std::logic_error("cannot start navigation while previous action is unresolved");
  }
  clear();
  state_ = State::Planning;
  stage_started_ = Clock::now();
  if (!ready()) {
    begin_stop(false, "navigation_not_ready");
    return;
  }
  Plan::Goal goal;
  goal.goal = target;
  goal.use_start = false;
  plan_pending_ = true;
  plan_terminal_ = false;
  const auto generation = generation_;
  rclcpp_action::Client<Plan>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](const PlanHandle::SharedPtr &handle) {
    on_plan_response(generation, handle);
  };
  options.result_callback = [this, generation](const PlanHandle::WrappedResult &result) {
    on_plan_result(generation, result);
  };
  planner_->async_send_goal(goal, options);
  event_("planning_started", "");
}

void NavigationClient::on_plan_response(std::uint64_t generation,
                                        const PlanHandle::SharedPtr &handle) {
  if (generation != generation_) {
    if (handle) planner_->async_cancel_goal(handle);
    return;
  }
  plan_pending_ = false;
  plan_handle_ = handle;
  if (!handle) {
    plan_terminal_ = true;
    if (state_ != State::Stopping) begin_stop(false, "planning_rejected");
  } else if (state_ == State::Stopping || state_ == State::Failed) {
    cancel_owned_goals();
  }
}

bool NavigationClient::valid_path(const nav_msgs::msg::Path &path) const {
  if (path.header.frame_id != "map" || path.poses.empty()) return false;
  const geometry_msgs::msg::PoseStamped *previous = nullptr;
  for (const auto &pose : path.poses) {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != "map") ||
        !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
        !valid_quaternion(pose.pose.orientation)) {
      return false;
    }
    if (previous && std::hypot(pose.pose.position.x - previous->pose.position.x,
                               pose.pose.position.y - previous->pose.position.y) > 1.0) return false;
    previous = &pose;
  }
  return true;
}

void NavigationClient::on_plan_result(std::uint64_t generation,
                                      const PlanHandle::WrappedResult &result) {
  if (generation != generation_) return;
  plan_terminal_ = true;
  if (state_ != State::Planning) return;
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != Plan::Result::NONE || !valid_path(result.result->path)) {
    begin_stop(false, "planning_failed");
    return;
  }
  send_follow(result.result->path);
}

void NavigationClient::send_follow(const nav_msgs::msg::Path &path) {
  Follow::Goal goal;
  goal.path = path;
  state_ = State::Following;
  stage_started_ = Clock::now();
  follow_pending_ = true;
  follow_terminal_ = false;
  const auto generation = generation_;
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](const FollowHandle::SharedPtr &handle) {
    on_follow_response(generation, handle);
  };
  options.feedback_callback = [this, generation](FollowHandle::SharedPtr,
                                                  const std::shared_ptr<const Follow::Feedback> feedback) {
    if (generation == generation_ && state_ == State::Following &&
        std::isfinite(feedback->distance_to_goal)) {
      distance_to_goal_ = feedback->distance_to_goal;
    }
  };
  options.result_callback = [this, generation](const FollowHandle::WrappedResult &result) {
    on_follow_result(generation, result);
  };
  controller_->async_send_goal(goal, options);
  event_("following_started", "");
}

void NavigationClient::on_follow_response(std::uint64_t generation,
                                          const FollowHandle::SharedPtr &handle) {
  if (generation != generation_) {
    if (handle) controller_->async_cancel_goal(handle);
    return;
  }
  follow_pending_ = false;
  follow_handle_ = handle;
  if (!handle) {
    follow_terminal_ = true;
    if (state_ != State::Stopping) begin_stop(false, "following_rejected");
  } else if (state_ == State::Stopping || state_ == State::Failed) {
    cancel_owned_goals();
  }
}

void NavigationClient::on_follow_result(std::uint64_t generation,
                                        const FollowHandle::WrappedResult &result) {
  if (generation != generation_) return;
  follow_terminal_ = true;
  if (state_ != State::Following) return;
  const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
                       result.result->error_code == Follow::Result::NONE;
  begin_stop(success, success ? "arrived" : "following_failed");
}

void NavigationClient::cancel_owned_goals() {
  if (plan_handle_ && !plan_terminal_ && !cancel_sent_plan_) {
    cancel_sent_plan_ = true;
    planner_->async_cancel_goal(plan_handle_);
  }
  if (follow_handle_ && !follow_terminal_ && !cancel_sent_follow_) {
    cancel_sent_follow_ = true;
    controller_->async_cancel_goal(follow_handle_);
  }
}

void NavigationClient::begin_stop(bool success, const std::string &reason) {
  if (state_ == State::Stopping) return;
  state_ = State::Stopping;
  success_requested_ = success;
  reason_ = reason;
  stop_started_ = Clock::now();
  cancel_owned_goals();
  event_("stopping", reason);
}

void NavigationClient::cancel(const std::string &reason) {
  begin_stop(false, reason);
}

void NavigationClient::poll() {
  if (state_ == State::Planning && elapsed(stage_started_) > limits_.planning_timeout) {
    begin_stop(false, "planning_timeout");
  } else if (state_ == State::Following) {
    if (!fresh_odometry()) begin_stop(false, "stale_odometry");
    else if (elapsed(stage_started_) > limits_.navigation_timeout) {
      begin_stop(false, "navigation_timeout");
    }
  }
  if (state_ != State::Stopping) return;
  const bool terminal = !plan_pending_ && !follow_pending_ && plan_terminal_ && follow_terminal_;
  // Нужна новая одометрия после запроса остановки: старое нулевое измерение недостаточно.
  if (terminal && stopped() && odom_received_at_ >= stop_started_) {
    stop_confirmed_ = true;
    state_ = success_requested_ ? State::Succeeded : State::Failed;
    event_(success_requested_ ? "navigation_succeeded" : "navigation_failed", reason_);
  } else if (elapsed(stop_started_) > limits_.stop_timeout) {
    state_ = State::Failed;
    reason_ = terminal ? "failure_to_stop" : "action_result_unknown";
    event_("navigation_fault", reason_);
  }
}

void NavigationClient::clear() {
  ++generation_;
  state_ = State::Idle;
  plan_handle_.reset();
  follow_handle_.reset();
  plan_pending_ = follow_pending_ = false;
  plan_terminal_ = follow_terminal_ = true;
  cancel_sent_plan_ = cancel_sent_follow_ = false;
  success_requested_ = stop_confirmed_ = false;
  reason_.clear();
}

}  // namespace capstone_mission
