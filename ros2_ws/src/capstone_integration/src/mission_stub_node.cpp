#include "capstone_integration/mission_stub_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "capstone_integration/episode_record.hpp"
#include "capstone_integration/path_builder.hpp"

namespace capstone_integration {

namespace {

using namespace std::chrono_literals;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

int count_true(const std::vector<bool> &values) {
  return static_cast<int>(std::count(values.begin(), values.end(), true));
}

}  // namespace

MissionStubNode::MissionStubNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mission_stub", options) {
  service_time_s_ = declare_parameter<double>("service_time_s", 2.0);
  mission_timeout_s_ = declare_parameter<double>("mission_timeout_s", 300.0);
  results_dir_ = declare_parameter<std::string>("results_dir", "/workspace/results");
  episode_id_ = declare_parameter<std::string>("episode_id", "stub-001");
  config_id_ = declare_parameter<std::string>("config_id", "SKELETON");
  scenario_id_ = declare_parameter<std::string>("scenario_id", "S1");
  jobs_file_ = declare_parameter<std::string>("jobs_file", "");
  auto_start_ = declare_parameter<bool>("auto_start", true);

  if (!(service_time_s_ >= 0.0) || !(mission_timeout_s_ > 0.0)) {
    throw std::invalid_argument("mission_stub: service_time_s >= 0 and mission_timeout_s > 0");
  }

  if (jobs_file_.empty()) {
    jobs_ = default_jobs();
  } else {
    std::ifstream in(jobs_file_);
    if (!in) {
      throw std::runtime_error("mission_stub: cannot read jobs file " + jobs_file_);
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    jobs_ = parse_jobs(buffer.str());
  }
  released_.assign(jobs_.size(), false);
  delivered_.assign(jobs_.size(), false);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", rclcpp::QoS(5).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
        on_pose(message);
      });
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/mission/status", rclcpp::QoS(10).reliable());

  path_client_ = rclcpp_action::create_client<ComputePath>(this, "/planning/compute_path");
  follow_client_ = rclcpp_action::create_client<FollowPath>(this, "/control/follow_path");

  timer_ = create_wall_timer(100ms, [this]() { tick(); });
  status_timer_ = create_wall_timer(1s, [this]() { publish_status(); });

  RCLCPP_WARN(get_logger(),
              "PLACEHOLDER G5: %zu jobs, earliest-due dispatch, no Behavior Tree and no recovery. "
              "Replace with capstone_mission.",
              jobs_.size());
}

void MissionStubNode::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
  pose_ = Pose2D{message.pose.pose.position.x, message.pose.pose.position.y,
                 yaw_from_quaternion(message.pose.pose.orientation)};
  has_pose_ = true;
}

void MissionStubNode::release_jobs(double elapsed_s) {
  for (std::size_t i = 0; i < jobs_.size(); ++i) {
    if (!released_[i] && jobs_[i].release_s <= elapsed_s) {
      released_[i] = true;
      RCLCPP_INFO(get_logger(), "job %s released at %.1f s", jobs_[i].id.c_str(), elapsed_s);
    }
  }
}

void MissionStubNode::tick() {
  if (finished_) {
    return;
  }
  if (auto_start_ && !started_) {
    start_time_ = now();
    started_ = true;
    RCLCPP_INFO(get_logger(), "episode %s started with %zu jobs", episode_id_.c_str(),
                jobs_.size());
  }
  const double elapsed = started_ ? (now() - start_time_).seconds() : 0.0;
  if (started_) {
    release_jobs(elapsed);
    if (elapsed > mission_timeout_s_) {
      finish("TIMEOUT", "episode limit reached");
      return;
    }
  }

  switch (state_) {
    case State::Init:
      if (auto_start_ && has_pose_ && path_client_->action_server_is_ready() &&
          follow_client_->action_server_is_ready()) {
        state_ = State::Idle;
      }
      break;
    case State::Idle:
      if (count_true(delivered_) == static_cast<int>(jobs_.size())) {
        finish("SUCCESS", "all jobs delivered");
      } else if (started_) {
        start_next_job();
      }
      break;
    case State::Planning:
    case State::Driving:
      break;
    case State::Service:
      if ((now() - service_start_).seconds() >= service_time_s_) {
        complete_service();
      }
      break;
    case State::Done:
      break;
  }
}

void MissionStubNode::start_next_job() {
  const std::size_t index = select_next_job(jobs_, released_, delivered_);
  if (index >= jobs_.size()) {
    return;
  }
  active_job_ = index;
  has_active_ = true;
  leg_ = Leg::Pickup;
  target_ = jobs_[index].pickup;
  active_target_name_ = "pickup " + jobs_[index].id;
  send_path_goal(target_);
  state_ = State::Planning;
}

void MissionStubNode::send_path_goal(const Pose2D &target) {
  const double heading = distance(pose_, target) > 0.05
                             ? std::atan2(target.y - pose_.y, target.x - pose_.x)
                             : pose_.yaw;
  ComputePath::Goal goal;
  goal.goal.header.frame_id = "map";
  goal.goal.header.stamp = now();
  goal.goal.pose.position.x = target.x;
  goal.goal.pose.position.y = target.y;
  goal.goal.pose.orientation = yaw_to_quaternion(heading);
  goal.start.header.frame_id = "map";
  goal.start.header.stamp = now();
  goal.start.pose.position.x = pose_.x;
  goal.start.pose.position.y = pose_.y;
  goal.start.pose.orientation = yaw_to_quaternion(pose_.yaw);
  goal.use_start = true;
  goal.planner_id = "integration_stub";

  PathClient::SendGoalOptions options;
  options.goal_response_callback = [this](const PathClient::GoalHandle::SharedPtr &handle) {
    if (!handle) {
      finish("ABORTED", "planning goal rejected");
    }
  };
  options.result_callback = [this](const PathWrappedResult &result) { on_path_result(result); };
  path_client_->async_send_goal(goal, options);
  RCLCPP_INFO(get_logger(), "planning to %s (%.1f, %.1f)", active_target_name_.c_str(), target.x,
              target.y);
}

void MissionStubNode::send_follow_goal(const nav_msgs::msg::Path &path) {
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = "integration_stub";
  goal.goal_checker_id = "integration_stub";
  goal.progress_checker_id = "integration_stub";

  FollowClient::SendGoalOptions options;
  options.goal_response_callback = [this](const FollowClient::GoalHandle::SharedPtr &handle) {
    if (!handle) {
      finish("ABORTED", "FollowPath goal rejected");
    }
  };
  options.result_callback = [this](const FollowWrappedResult &result) { on_follow_result(result); };
  follow_client_->async_send_goal(goal, options);
}

void MissionStubNode::on_path_result(const PathWrappedResult &result) {
  if (state_ != State::Planning) {
    return;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != ComputePath::Result::NONE || result.result->path.poses.empty()) {
    const std::string message =
        result.result ? result.result->error_msg : "planning failed without a result";
    finish("ABORTED", "planning failed: " + message);
    return;
  }
  path_msg_ = result.result->path;
  distance_m_ += path_length(path_points());
  send_follow_goal(path_msg_);
  state_ = State::Driving;
}

void MissionStubNode::on_follow_result(const FollowWrappedResult &result) {
  if (state_ != State::Driving) {
    return;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != FollowPath::Result::NONE) {
    const uint16_t code = result.result ? result.result->error_code : FollowPath::Result::UNKNOWN;
    const std::string message = result.result ? result.result->error_msg : "no result";
    finish(code == FollowPath::Result::CONTROLLER_TIMED_OUT ? "TIMEOUT" : "ABORTED",
           "following failed: " + message);
    return;
  }
  service_start_ = now();
  state_ = State::Service;
  RCLCPP_INFO(get_logger(), "reached %s, servicing %.1f s", active_target_name_.c_str(),
              service_time_s_);
}

void MissionStubNode::complete_service() {
  if (!has_active_) {
    state_ = State::Idle;
    return;
  }
  if (leg_ == Leg::Pickup) {
    leg_ = Leg::Dropoff;
    target_ = jobs_[active_job_].dropoff;
    active_target_name_ = "dropoff " + jobs_[active_job_].id;
    send_path_goal(target_);
    state_ = State::Planning;
    return;
  }

  delivered_[active_job_] = true;
  const double elapsed = (now() - start_time_).seconds();
  const double due = jobs_[active_job_].release_s + jobs_[active_job_].due_s;
  if (elapsed <= due) {
    ++on_time_;
  }
  RCLCPP_INFO(get_logger(), "job %s delivered at %.1f s (due %.1f s)", jobs_[active_job_].id.c_str(),
              elapsed, due);
  has_active_ = false;
  state_ = State::Idle;
}

void MissionStubNode::finish(const std::string &terminal_reason, const std::string &message) {
  if (finished_) {
    return;
  }
  finished_ = true;
  state_ = State::Done;
  terminal_message_ = message;
  const int delivered_count = count_true(delivered_);
  if (delivered_count == static_cast<int>(jobs_.size())) {
    terminal_reason_ = "SUCCESS";
  } else {
    terminal_reason_ = terminal_reason == "SUCCESS" ? "TIMEOUT" : terminal_reason;
  }
  write_episode();
  publish_status();
  RCLCPP_INFO(get_logger(), "episode %s finished: %s (%s, delivered %d/%zu)", episode_id_.c_str(),
              terminal_reason_.c_str(), message.c_str(), delivered_count, jobs_.size());
}

std::vector<Pose2D> MissionStubNode::path_points() const {
  std::vector<Pose2D> points;
  points.reserve(path_msg_.poses.size());
  for (const auto &pose : path_msg_.poses) {
    points.push_back(Pose2D{pose.pose.position.x, pose.pose.position.y,
                            yaw_from_quaternion(pose.pose.orientation)});
  }
  return points;
}

void MissionStubNode::write_episode() {
  EpisodeRecord record;
  record.episode_id = episode_id_;
  record.config_id = config_id_;
  record.scenario_id = scenario_id_;
  record.terminal_reason = terminal_reason_;
  record.elapsed_sim_s = started_ ? (now() - start_time_).seconds() : 0.0;
  record.delivered = count_true(delivered_);
  record.released = count_true(released_);
  record.on_time = on_time_;
  record.collision_count = 0;
  record.distance_m = distance_m_;
  record.deadline_misses = std::max(0, record.released - record.on_time);

  try {
    const std::filesystem::path directory =
        std::filesystem::path(results_dir_) / record.episode_id;
    std::filesystem::create_directories(directory);
    const std::filesystem::path file = directory / "episode.csv";
    std::ofstream out(file, std::ios::app);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", file.c_str());
      return;
    }
    if (out.tellp() == 0) {
      out << csv_header() << '\n';
    }
    out << to_csv_row(record) << '\n';
    RCLCPP_INFO(get_logger(), "episode written to %s", file.c_str());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "cannot write the episode: %s", error.what());
  }
}

void MissionStubNode::publish_status() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "capstone_mission/mission";
  status.hardware_id = "capstone_integration";
  status.level = (state_ == State::Done && terminal_reason_ != "SUCCESS")
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = std::string("state=") + state_name(state_) +
                   " leg=" + (has_active_ ? leg_name(leg_) : "-") +
                   " delivered=" + std::to_string(count_true(delivered_)) + "/" +
                   std::to_string(jobs_.size());
  diagnostic_msgs::msg::KeyValue episode;
  episode.key = "episode_id";
  episode.value = episode_id_;
  status.values.push_back(episode);
  array.status.push_back(status);
  status_pub_->publish(array);
}

const char *MissionStubNode::state_name(State state) {
  switch (state) {
    case State::Init:
      return "INIT";
    case State::Idle:
      return "IDLE";
    case State::Planning:
      return "PLANNING";
    case State::Driving:
      return "DRIVING";
    case State::Service:
      return "SERVICE";
    case State::Done:
      return "DONE";
  }
  return "UNKNOWN";
}

const char *MissionStubNode::leg_name(Leg leg) {
  return leg == Leg::Pickup ? "PICKUP" : "DROPOFF";
}

}  // namespace capstone_integration
