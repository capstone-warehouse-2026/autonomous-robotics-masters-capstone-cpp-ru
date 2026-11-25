#include "capstone_fleet_runtime/robot_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capstone_fleet_runtime {
namespace {
bool valid_orientation(const geometry_msgs::msg::Quaternion &q) {
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(norm) && std::abs(norm - 1) < 0.01;
}
}  // namespace

RobotNavigator::RobotNavigator(rclcpp::Node &node, std::string robot_id, double timeout)
    : node_(node), robot_id_(std::move(robot_id)), timeout_(timeout) {
  if (!std::isfinite(timeout_) || timeout_ <= 0) throw std::invalid_argument("invalid edge timeout");
  const std::string prefix = "/" + robot_id_;
  // Центральный координатор явно выбирает робота; TF каждого Nav2 остаётся в его namespace.
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(prefix + "/state/odom", 5,
      [this](const nav_msgs::msg::Odometry &message) { on_odometry(message); });
  pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      prefix + "/localization/pose", 5,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped &message) { on_pose(message); });
  client_ = rclcpp_action::create_client<Follow>(&node_, prefix + "/control/follow_path");
}

bool RobotNavigator::valid_stamp(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}

void RobotNavigator::on_odometry(const nav_msgs::msg::Odometry &message) {
  if (message.header.frame_id != "odom" || message.child_frame_id != "base_link" ||
      !valid_stamp(message.header.stamp) || !std::isfinite(message.twist.twist.linear.x) ||
      !std::isfinite(message.twist.twist.angular.z)) return;
  const auto wall = Clock::now();
  const double now = node_.now().seconds();
  // Продвинутый после /clock образец может содержать движение; наблюдаем его до
  // приёма следующего, чтобы пакет moving → zero не исчез между вызовами update().
  odometry_.advance(now, wall);
  update_stopped_interval();
  odometry_.receive(message, rclcpp::Time(message.header.stamp).seconds(), now, wall);
  update_stopped_interval();
}

void RobotNavigator::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
  if (message.header.frame_id != "map" || !valid_stamp(message.header.stamp) ||
      !std::isfinite(message.pose.pose.position.x) || !std::isfinite(message.pose.pose.position.y) ||
      !valid_orientation(message.pose.pose.orientation)) return;
  pose_.receive(message, rclcpp::Time(message.header.stamp).seconds(),
                node_.now().seconds(), Clock::now());
}

void RobotNavigator::update() {
  const auto wall = Clock::now();
  const double now = node_.now().seconds();
  odometry_.advance(now, wall);
  pose_.advance(now, wall);
  update_stopped_interval();
  if (reservation_active_ && !cancel_requested_ && inputs_fresh() &&
      distance_from_reserved_edge() > 0.12) {
    reason_ = "left_reserved_corridor";
    state_ = ActionState::Failed;
    cancel_requested_ = true;
    cancel_owned_goal();
    return;
  }
  if ((state_ == ActionState::Pending || state_ == ActionState::Active ||
       state_ == ActionState::Succeeded) &&
      std::chrono::duration<double>(wall - goal_started_).count() > timeout_ && !edge_complete()) {
    reason_ = "edge_timeout";
    state_ = ActionState::Failed;
    cancel_requested_ = true;
    cancel_owned_goal();
  }
}

bool RobotNavigator::inputs_fresh() const {
  const auto wall = Clock::now();
  const double now = node_.now().seconds();
  return odometry_.fresh(now, wall) && pose_.fresh(now, wall);
}
bool RobotNavigator::ready() const { return client_->action_server_is_ready() && inputs_fresh(); }
double RobotNavigator::x() const { return pose_.sample()->message.pose.pose.position.x; }
double RobotNavigator::y() const { return pose_.sample()->message.pose.pose.position.y; }

void RobotNavigator::follow_edge(double start_x, double start_y, double goal_x, double goal_y) {
  if (!terminal()) throw std::logic_error("cannot replace an unresolved FollowPath goal");
  if (!ready() || std::hypot(x() - start_x, y() - start_y) > 0.12) {
    throw std::runtime_error("robot is not ready at its reserved edge start: " + robot_id_);
  }
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  start_x_ = start_x;
  start_y_ = start_y;
  reservation_active_ = true;
  reason_.clear();
  cancel_requested_ = cancel_sent_ = false;
  stop_requested_stamp_ = 0;
  stop_requested_wall_.reset();
  response_pending_ = true;
  terminal_received_ = false;
  state_ = ActionState::Pending;
  goal_started_ = Clock::now();
  const auto generation = ++generation_;
  const double yaw = std::atan2(goal_y - start_y, goal_x - start_x);
  Follow::Goal goal;
  goal.controller_id = "FollowPath";
  goal.goal_checker_id = "goal";
  goal.progress_checker_id = "progress";
  goal.path.header.frame_id = "map";
  goal.path.header.stamp = node_.now();
  const double actual_x = x(), actual_y = y();
  const int samples = std::max(2, static_cast<int>(std::ceil(
      std::hypot(goal_x - actual_x, goal_y - actual_y) / 0.1)));
  for (int i = 0; i <= samples; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = goal.path.header;
    // Геометрическая цель в map постоянна: goal checker должен брать актуальную
    // map→odom TF после коррекций AMCL, как и RPP при преобразовании пути.
    pose.header.stamp = rclcpp::Time(0);
    const double fraction = static_cast<double>(i) / samples;
    pose.pose.position.x = actual_x + fraction * (goal_x - actual_x);
    pose.pose.position.y = actual_y + fraction * (goal_y - actual_y);
    pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.orientation.w = std::cos(yaw / 2);
    goal.path.poses.push_back(pose);
  }
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](const GoalHandle::SharedPtr &handle) {
    on_goal_response(generation, handle);
  };
  options.result_callback = [this, generation](const GoalHandle::WrappedResult &result) {
    on_result(generation, result);
  };
  options.feedback_callback = [this, generation](GoalHandle::SharedPtr,
                                                const std::shared_ptr<const Follow::Feedback> feedback) {
    if (generation == generation_ && std::isfinite(feedback->distance_to_goal)) {
      distance_remaining_ = feedback->distance_to_goal;
    }
  };
  client_->async_send_goal(goal, options);
}

void RobotNavigator::wait_at(double x, double y) {
  if (!terminal()) throw std::logic_error("wait cannot replace an unresolved goal");
  goal_x_ = x;
  goal_y_ = y;
  start_x_ = x;
  start_y_ = y;
  reservation_active_ = true;
  cancel_requested_ = cancel_sent_ = false;
  stop_requested_stamp_ = 0;
  stop_requested_wall_.reset();
  state_ = ActionState::Idle;
  reason_.clear();
}

void RobotNavigator::on_goal_response(std::uint64_t generation, const GoalHandle::SharedPtr &handle) {
  if (generation != generation_) {
    if (handle) client_->async_cancel_goal(handle);
    return;
  }
  response_pending_ = false;
  if (!handle) {
    terminal_received_ = true;
    result_received_ = Clock::now();
    result_stamp_ = node_.now().seconds();
    state_ = ActionState::Failed;
    reason_ = "goal_rejected";
    return;
  }
  handle_ = handle;
  state_ = cancel_requested_ ? ActionState::Cancelling : ActionState::Active;
  if (cancel_requested_) cancel_owned_goal();
}

void RobotNavigator::on_result(std::uint64_t generation, const GoalHandle::WrappedResult &result) {
  if (generation != generation_) return;
  terminal_received_ = true;
  result_received_ = Clock::now();
  result_stamp_ = node_.now().seconds();
  handle_.reset();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
      result.result->error_code == 0) {
    state_ = ActionState::Succeeded;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED && cancel_requested_) {
    state_ = ActionState::Cancelled;
  } else {
    state_ = ActionState::Failed;
    reason_ = "follow_failed_" + std::to_string(result.result ? result.result->error_code : -1);
  }
}

void RobotNavigator::cancel_owned_goal() {
  if (!handle_ || terminal_received_ || cancel_sent_) return;
  cancel_sent_ = true;
  client_->async_cancel_goal(handle_);
}

void RobotNavigator::cancel() {
  if (!stop_requested_wall_) {
    stop_requested_stamp_ = node_.now().seconds();
    stop_requested_wall_ = Clock::now();
  }
  cancel_requested_ = true;
  if (!terminal()) state_ = ActionState::Cancelling;
  cancel_owned_goal();
}

void RobotNavigator::update_stopped_interval() {
  if (!odometry_.sample()) {
    stopped_since_ = last_odom_stamp_ = -1;
    stopped_since_wall_.reset();
    last_odom_received_.reset();
    return;
  }
  const auto &sample = *odometry_.sample();
  const bool gap = last_odom_received_ &&
      (sample.stamp - last_odom_stamp_ > 0.3 ||
       std::chrono::duration<double>(sample.received - *last_odom_received_).count() > 0.3);
  if (sample.stamp < last_odom_stamp_ || gap) {
    stopped_since_ = -1;
    stopped_since_wall_.reset();
  }
  last_odom_stamp_ = sample.stamp;
  last_odom_received_ = sample.received;
  const auto &twist = sample.message.twist.twist;
  if (std::abs(twist.linear.x) >= 0.03 || std::abs(twist.angular.z) >= 0.05) {
    stopped_since_ = -1;
    stopped_since_wall_.reset();
  } else if (stopped_since_ < 0) {
    stopped_since_ = sample.stamp;
    stopped_since_wall_ = sample.received;
  }
}

double RobotNavigator::distance_from_reserved_edge() const {
  const double dx = goal_x_ - start_x_, dy = goal_y_ - start_y_;
  const double length_squared = dx * dx + dy * dy;
  const double fraction = length_squared > 0 ?
      std::clamp(((x() - start_x_) * dx + (y() - start_y_) * dy) / length_squared, 0.0, 1.0) : 0.0;
  return std::hypot(x() - start_x_ - fraction * dx, y() - start_y_ - fraction * dy);
}

bool RobotNavigator::stopped() const {
  return inputs_fresh() && stopped_since_ >= 0 && stopped_since_wall_ &&
      odometry_.sample()->stamp - stopped_since_ >= 0.2 &&
      std::chrono::duration<double>(odometry_.sample()->received - *stopped_since_wall_).count() >= 0.2;
}

bool RobotNavigator::settled_at(double x, double y) const {
  return stopped() && std::hypot(this->x() - x, this->y() - y) <= 0.12;
}

bool RobotNavigator::stopped_after(double stamp, Clock::time_point received) const {
  return stopped() &&
      odometry_.sample()->stamp - std::max(stopped_since_, stamp) >= 0.2 &&
      std::chrono::duration<double>(odometry_.sample()->received -
          std::max(*stopped_since_wall_, received)).count() >= 0.2;
}

bool RobotNavigator::stop_complete() const {
  // Тишина до cancel/result не доказывает остановку после последней команды Nav2.
  return terminal() && stopped_after(std::max(result_stamp_, stop_requested_stamp_),
      std::max(result_received_, stop_requested_wall_.value_or(Clock::time_point{})));
}

bool RobotNavigator::edge_complete() const {
  if (state_ == ActionState::Idle) return settled_at(goal_x_, goal_y_);
  return state_ == ActionState::Succeeded && terminal() && settled_at(goal_x_, goal_y_) &&
         stopped_after(result_stamp_, result_received_);
}
}  // namespace capstone_fleet_runtime
