#include "capstone_planning/planning_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/time.h>

#include "capstone_planning/map_fusion.hpp"
#include "capstone_planning/planner.hpp"

namespace capstone_planning {

namespace {

constexpr std::size_t kLatencyWindow = 1000;
constexpr double kPathCheckPeriod = 0.5;  // s, steady clock, re-check of the last path on new obstacles

using Result = nav2_msgs::action::ComputePathToPose::Result;

double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double pos = q * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = std::min(v.size() - 1, lo + 1);
  return v[lo] + (pos - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

uint16_t error_code_of(PlanStatus status) {
  switch (status) {
    case PlanStatus::kSuccess: return Result::NONE;
    case PlanStatus::kStartOutsideMap: return Result::START_OUTSIDE_MAP;
    case PlanStatus::kGoalOutsideMap: return Result::GOAL_OUTSIDE_MAP;
    case PlanStatus::kStartBlocked: return Result::START_OCCUPIED;
    case PlanStatus::kGoalBlocked: return Result::GOAL_OCCUPIED;
    case PlanStatus::kNoPath: return Result::NO_VALID_PATH;
    case PlanStatus::kTimeout: return Result::TIMEOUT;
    case PlanStatus::kCanceled: return Result::UNKNOWN;
  }
  return Result::UNKNOWN;
}

Pose2D pose_from_message(const geometry_msgs::msg::Pose &pose) {
  return {pose.position.x, pose.position.y,
          yaw_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)};
}

bool finite_pose(const geometry_msgs::msg::Pose &p) {
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
         std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
         std::isfinite(p.orientation.w) &&
         (p.orientation.x * p.orientation.x + p.orientation.y * p.orientation.y +
          p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w) > 1e-6;
}

Result failure(uint16_t code, const std::string &message) {
  Result result;
  result.error_code = code;
  result.error_msg = message;
  return result;
}

}  // namespace

double yaw_from_quaternion(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Grid grid_from_message(const nav_msgs::msg::OccupancyGrid &message) {
  Grid grid;
  grid.width = static_cast<int>(message.info.width);
  grid.height = static_cast<int>(message.info.height);
  grid.resolution = message.info.resolution;
  grid.origin = pose_from_message(message.info.origin);
  grid.data = message.data;
  if (!finite_pose(message.info.origin) || !grid.valid()) {
    throw std::invalid_argument("invalid OccupancyGrid: empty, non-finite origin/resolution or data size mismatch");
  }
  return grid;
}

PlanningNode::PlanningNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("planning", options),
      method_(declare_parameter<std::string>("method", "classical")),
      map_frame_(declare_parameter<std::string>("map_frame", "map")) {
  if (method_ != "classical" && method_ != "learned") {
    throw std::invalid_argument("method must be 'classical' or 'learned', got '" + method_ + "'");
  }
  cost_params_.robot_radius = declare_parameter<double>("robot_radius", cost_params_.robot_radius);
  cost_params_.inflation_margin = declare_parameter<double>("inflation_margin", cost_params_.inflation_margin);
  cost_params_.clearance_band = declare_parameter<double>("cost.clearance_band", cost_params_.clearance_band);
  cost_params_.clearance_weight = declare_parameter<double>("cost.clearance_weight", cost_params_.clearance_weight);
  cost_params_.observed_band = declare_parameter<double>("cost.observed_band", cost_params_.observed_band);
  cost_params_.observed_weight = declare_parameter<double>("cost.observed_weight", cost_params_.observed_weight);
  cost_params_.unknown_is_obstacle = declare_parameter<bool>("unknown_is_obstacle", cost_params_.unknown_is_obstacle);
  cost_params_.unknown_weight = declare_parameter<double>("cost.unknown_weight", cost_params_.unknown_weight);
  cost_params_.validate();
  planning_timeout_ = declare_parameter<double>("planning_timeout", planning_timeout_);
  max_obstacles_age_ = declare_parameter<double>("max_obstacles_age", max_obstacles_age_);
  max_pose_age_ = declare_parameter<double>("max_pose_age", max_pose_age_);
  tf_timeout_ = declare_parameter<double>("tf_timeout", tf_timeout_);
  start_escape_radius_ = declare_parameter<double>("start_escape_radius", start_escape_radius_);
  require_obstacles_ = declare_parameter<bool>("require_obstacles", require_obstacles_);
  for (double v : {planning_timeout_, max_obstacles_age_, max_pose_age_, tf_timeout_}) {
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument("planning_timeout, max_obstacles_age, max_pose_age and tf_timeout must be > 0");
    }
  }
  if (!std::isfinite(start_escape_radius_) || start_escape_radius_ < 0.0) {
    throw std::invalid_argument("start_escape_radius must be >= 0");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS of docs/INTERFACES.md.
  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) { on_map(m); });
  obstacles_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/perception/obstacles", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) { on_obstacles(m); });
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", rclcpp::QoS(5).reliable().durability_volatile(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { on_pose(m); });
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_diagnostics(); });

  worker_ = std::thread([this] { worker_loop(); });
  action_server_ = rclcpp_action::create_server<ComputePath>(
      this, "/planning/compute_path",
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ComputePath::Goal> goal) {
        return on_goal(uuid, std::move(goal));
      },
      [this](std::shared_ptr<GoalHandle> goal_handle) { return on_cancel(std::move(goal_handle)); },
      [this](std::shared_ptr<GoalHandle> goal_handle) { on_accepted(std::move(goal_handle)); });

  if (method_ == "learned") {
    RCLCPP_WARN(get_logger(), "method=learned: the learned cost is not implemented yet, the classical cost is used "
                              "as its fallback");
  }
  RCLCPP_INFO(get_logger(), "planning started: method=%s, frame %s, inflation radius %.3f m", method_.c_str(),
              map_frame_.c_str(), cost_params_.inflation_radius());
}

PlanningNode::~PlanningNode() {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    stopping_ = true;
  }
  goal_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool PlanningNode::has_map() const {
  std::lock_guard<std::mutex> lock(inputs_mutex_);
  return static_map_ != nullptr;
}

// --- inputs ---------------------------------------------------------------------------------------

void PlanningNode::on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
  try {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      throw std::invalid_argument("/map frame '" + message->header.frame_id + "' is not " + map_frame_);
    }
    auto grid = std::make_shared<const Grid>(grid_from_message(*message));
    std::lock_guard<std::mutex> lock(inputs_mutex_);
    static_map_ = std::move(grid);
    RCLCPP_INFO(get_logger(), "map received: %d x %d cells, %.3f m", static_map_->width, static_map_->height,
                static_map_->resolution);
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> lock(inputs_mutex_);
    last_input_error_ = std::string("/map rejected: ") + error.what();
    RCLCPP_ERROR(get_logger(), "%s", last_input_error_.c_str());
  }
}

void PlanningNode::on_obstacles(const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
  try {
    if (message->header.frame_id.empty()) {
      throw std::invalid_argument("empty frame_id");
    }
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      throw std::invalid_argument("zero stamp");
    }
    auto local = std::make_shared<LocalGrid>();
    local->grid = grid_from_message(*message);
    local->frame = message->header.frame_id;
    local->stamp = stamp;
    std::lock_guard<std::mutex> lock(inputs_mutex_);
    local_grid_ = std::move(local);
  } catch (const std::exception &error) {
    std::lock_guard<std::mutex> lock(inputs_mutex_);
    last_input_error_ = std::string("/perception/obstacles rejected: ") + error.what();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", last_input_error_.c_str());
    return;
  }
  const auto now_steady = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now_steady - last_path_check_).count() >= kPathCheckPeriod) {
    last_path_check_ = now_steady;
    check_last_path();
  }
}

void PlanningNode::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
  std::lock_guard<std::mutex> lock(inputs_mutex_);
  pose_ = *message;
}

// --- action server --------------------------------------------------------------------------------

rclcpp_action::GoalResponse PlanningNode::on_goal(const rclcpp_action::GoalUUID &,
                                                  std::shared_ptr<const ComputePath::Goal>) {
  // Every goal is accepted; invalid ones are aborted with an error code by the worker, so that the
  // client always gets an explicit reason.
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanningNode::on_cancel(std::shared_ptr<GoalHandle>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanningNode::on_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  std::shared_ptr<GoalHandle> replaced;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    replaced = std::move(pending_goal_);
    pending_goal_ = std::move(goal_handle);
    preempt_ = true;  // stops the running search, if any
  }
  goal_cv_.notify_one();
  if (replaced && replaced->is_active()) {
    replaced->abort(std::make_shared<Result>(failure(Result::UNKNOWN, "preempted by a newer goal")));
  }
}

void PlanningNode::worker_loop() {
  while (true) {
    std::shared_ptr<GoalHandle> goal_handle;
    {
      std::unique_lock<std::mutex> lock(goal_mutex_);
      goal_cv_.wait(lock, [this] { return stopping_.load() || pending_goal_ != nullptr; });
      if (stopping_) {
        return;
      }
      goal_handle = std::move(pending_goal_);
      preempt_ = false;
    }
    auto result = std::make_shared<Result>();
    if (!goal_handle->is_canceling()) {
      *result = plan(*goal_handle->get_goal(),
                     [this, &goal_handle] { return preempt_.load() || stopping_.load() || goal_handle->is_canceling(); });
    }
    try {
      if (goal_handle->is_canceling()) {
        result->path = nav_msgs::msg::Path();
        result->error_code = Result::UNKNOWN;
        result->error_msg = "canceled";
        goal_handle->canceled(result);
      } else if (result->error_code == Result::NONE) {
        goal_handle->succeed(result);
      } else {
        if (preempt_) {
          result->error_msg = "preempted by a newer goal";
        }
        goal_handle->abort(result);
      }
    } catch (const std::exception &error) {
      // The goal may have been terminated concurrently (e.g. shutdown); nothing else to do.
      RCLCPP_WARN(get_logger(), "could not report the goal result: %s", error.what());
    }
  }
}

// --- planning -------------------------------------------------------------------------------------

std::optional<Pose2D> PlanningNode::to_map_frame(const geometry_msgs::msg::PoseStamped &pose, std::string &error) {
  if (!finite_pose(pose.pose)) {
    error = "pose has NaN/Inf or an invalid quaternion";
    return std::nullopt;
  }
  const Pose2D in_frame = pose_from_message(pose.pose);
  if (pose.header.frame_id == map_frame_) {
    return in_frame;
  }
  if (pose.header.frame_id.empty()) {
    error = "pose has an empty frame_id";
    return std::nullopt;
  }
  try {
    const auto t = tf_buffer_->lookupTransform(map_frame_, pose.header.frame_id, tf2::TimePointZero,
                                               tf2::durationFromSec(tf_timeout_));
    const Pose2D map_from_frame{t.transform.translation.x, t.transform.translation.y,
                                yaw_from_quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                    t.transform.rotation.z, t.transform.rotation.w)};
    return compose(map_from_frame, in_frame);
  } catch (const std::exception &e) {
    error = "no transform " + map_frame_ + " <- " + pose.header.frame_id + ": " + e.what();
    return std::nullopt;
  }
}

std::shared_ptr<const CostMap> PlanningNode::build_cost_map(std::string &error, uint16_t &error_code) {
  std::shared_ptr<const Grid> static_map;
  std::shared_ptr<const LocalGrid> local;
  {
    std::lock_guard<std::mutex> lock(inputs_mutex_);
    static_map = static_map_;
    local = local_grid_;
  }
  error_code = Result::UNKNOWN;
  if (!static_map) {
    error = "no /map received";
    return nullptr;
  }
  const bool fresh = local && (now() - local->stamp).seconds() <= max_obstacles_age_ &&
                     (now() - local->stamp).seconds() >= -max_obstacles_age_;
  Pose2D map_from_local;
  bool use_local = fresh;
  if (fresh && local->frame != map_frame_) {
    try {
      const auto t = tf_buffer_->lookupTransform(map_frame_, local->frame, local->stamp,
                                                 rclcpp::Duration::from_seconds(tf_timeout_));
      map_from_local = {t.transform.translation.x, t.transform.translation.y,
                        yaw_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                                            t.transform.rotation.w)};
    } catch (const std::exception &e) {
      use_local = false;
      if (require_obstacles_) {
        error_code = Result::TF_ERROR;
        error = "no transform " + map_frame_ + " <- " + local->frame + " for /perception/obstacles: " + e.what();
        return nullptr;
      }
    }
  }
  if (!fresh && require_obstacles_) {
    // docs/INTERFACES.md: do not plan on an old obstacle map.
    error = local ? "/perception/obstacles is stale" : "no /perception/obstacles received";
    return nullptr;
  }
  try {
    const FusedMap fused = fuse_maps(*static_map, use_local ? &local->grid : nullptr, map_from_local);
    return std::make_shared<const CostMap>(fused, cost_params_);
  } catch (const std::exception &e) {
    error = std::string("cost map: ") + e.what();
    return nullptr;
  }
}

PlanningNode::ComputePath::Result PlanningNode::plan(const ComputePath::Goal &goal,
                                                     const std::function<bool()> &cancel) {
  const auto started = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&started] {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  };
  std::size_t expanded = 0;
  const auto finish = [&](Result result) {
    const double ms = elapsed_ms();
    result.planning_time = rclcpp::Duration::from_seconds(ms / 1000.0);
    record(result, ms, expanded);
    return result;
  };

  // Start: the request's start or the latest localization pose.
  std::string error;
  std::optional<Pose2D> start;
  if (goal.use_start) {
    start = to_map_frame(goal.start, error);
    if (!start) return finish(failure(Result::TF_ERROR, "start: " + error));
  } else {
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose;
    {
      std::lock_guard<std::mutex> lock(inputs_mutex_);
      pose = pose_;
    }
    if (!pose) return finish(failure(Result::TF_ERROR, "no /localization/pose received and use_start is false"));
    const rclcpp::Time stamp(pose->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age > max_pose_age_ || age < -max_pose_age_) {
      return finish(failure(Result::TF_ERROR, "/localization/pose is stale (" + std::to_string(age) + " s)"));
    }
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = pose->header;
    stamped.pose = pose->pose.pose;
    start = to_map_frame(stamped, error);
    if (!start) return finish(failure(Result::TF_ERROR, "start: " + error));
  }
  const std::optional<Pose2D> target = to_map_frame(goal.goal, error);
  if (!target) return finish(failure(Result::TF_ERROR, "goal: " + error));

  uint16_t map_error_code = Result::UNKNOWN;
  const auto cost_map = build_cost_map(error, map_error_code);
  if (!cost_map) return finish(failure(map_error_code, error));
  if (method_ == "learned") {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++learned_fallbacks_;  // learned cost not implemented: classical fallback
  }

  SearchOptions options;
  options.deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(planning_timeout_));
  options.should_cancel = cancel;
  options.start_escape_radius = start_escape_radius_;
  const PlannedPath planned = plan_path(*cost_map, *start, *target, options);
  expanded = planned.expanded;
  if (planned.status != PlanStatus::kSuccess) {
    std::string message = to_string(planned.status);
    if (planned.status == PlanStatus::kTimeout) {
      message += " after " + std::to_string(planning_timeout_) + " s";
    }
    return finish(failure(error_code_of(planned.status), message));
  }

  Result result;
  result.error_code = Result::NONE;
  result.path.header.frame_id = map_frame_;
  result.path.header.stamp = now();
  result.path.poses.reserve(planned.poses.size());
  for (const Pose2D &p : planned.poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.path.header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.orientation.z = std::sin(0.5 * p.yaw);
    pose.pose.orientation.w = std::cos(0.5 * p.yaw);
    result.path.poses.push_back(pose);
  }
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    last_length_ = planned.length;
    last_min_clearance_ = planned.min_clearance;
    last_path_ = planned.poses;
    last_path_blocked_ = false;
  }
  return finish(result);
}

void PlanningNode::record(const Result &result, double latency_ms, std::size_t expanded) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  ++requests_;
  latencies_ms_.push_back(latency_ms);
  if (latencies_ms_.size() > kLatencyWindow) latencies_ms_.pop_front();
  if (latency_ms > planning_timeout_ * 1000.0) ++deadline_misses_;
  last_expanded_ = expanded;
  if (result.error_code == Result::NONE) {
    ++successes_;
  } else {
    ++failures_by_code_[result.error_code];
    last_error_ = result.error_msg;
    RCLCPP_WARN(get_logger(), "planning failed (code %u): %s", static_cast<unsigned>(result.error_code),
                result.error_msg.c_str());
  }
}

void PlanningNode::check_last_path() {
  std::vector<Pose2D> path;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (last_path_.empty() || last_path_blocked_) return;
    path = last_path_;
  }
  std::string error;
  uint16_t code = 0;
  const auto cost_map = build_cost_map(error, code);
  if (!cost_map || !path_blocked(*cost_map, path)) return;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    last_path_blocked_ = true;
    ++path_blocked_events_;
  }
  RCLCPP_WARN(get_logger(), "the last planned path is blocked by the updated map: replanning needed");
  publish_diagnostics();  // event, not only the 1 Hz report
}

void PlanningNode::publish_diagnostics() {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "planning";
  status.hardware_id = "capstone_planning";
  bool have_map = false;
  std::string obstacles_age = "unavailable";
  std::string input_error;
  {
    std::lock_guard<std::mutex> lock(inputs_mutex_);
    have_map = static_map_ != nullptr;
    if (local_grid_) obstacles_age = std::to_string((now() - local_grid_->stamp).seconds());
    input_error = last_input_error_;
  }
  const auto value = [](const std::string &key, const std::string &text) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = text;
    return pair;
  };
  std::lock_guard<std::mutex> lock(stats_mutex_);
  const std::vector<double> latencies(latencies_ms_.begin(), latencies_ms_.end());
  if (!have_map) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "waiting for /map";
  } else if (last_path_blocked_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "last path blocked by the updated map: replan";
  } else if (method_ == "learned") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "learned cost not implemented: classical fallback";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ok";
  }
  std::string failures;
  for (const auto &[code, count] : failures_by_code_) {
    failures += (failures.empty() ? "" : ",") + std::to_string(code) + ":" + std::to_string(count);
  }
  status.values = {value("method", method_),
                   value("requests", std::to_string(requests_)),
                   value("successes", std::to_string(successes_)),
                   value("failures_by_code", failures),
                   value("deadline_misses", std::to_string(deadline_misses_)),
                   value("latency_ms_p50", std::to_string(percentile(latencies, 0.50))),
                   value("latency_ms_p95", std::to_string(percentile(latencies, 0.95))),
                   value("latency_ms_p99", std::to_string(percentile(latencies, 0.99))),
                   value("last_expanded", std::to_string(last_expanded_)),
                   value("last_path_length_m", std::to_string(last_length_)),
                   value("last_min_clearance_m", std::to_string(last_min_clearance_)),
                   value("path_blocked_events", std::to_string(path_blocked_events_)),
                   value("learned_fallbacks", std::to_string(learned_fallbacks_)),
                   value("obstacles_age_s", obstacles_age),
                   value("last_error", last_error_),
                   value("last_input_error", input_error)};
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace capstone_planning
