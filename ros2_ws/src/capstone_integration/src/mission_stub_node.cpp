#include "capstone_integration/mission_stub_node.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
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

}  // namespace

MissionStubNode::MissionStubNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mission_stub", options) {
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  goal_x_ = declare_parameter<double>("goal_x", -8.8);
  goal_y_ = declare_parameter<double>("goal_y", 7.5);
  goal_yaw_ = declare_parameter<double>("goal_yaw", 1.5707963267948966);
  service_time_s_ = declare_parameter<double>("service_time_s", 2.0);
  mission_timeout_s_ = declare_parameter<double>("mission_timeout_s", 300.0);
  results_dir_ = declare_parameter<std::string>("results_dir", "/workspace/results");
  episode_id_ = declare_parameter<std::string>("episode_id", "stub-001");
  config_id_ = declare_parameter<std::string>("config_id", "SKELETON");
  scenario_id_ = declare_parameter<std::string>("scenario_id", "S1");
  auto_start_ = declare_parameter<bool>("auto_start", true);

  if (!(service_time_s_ >= 0.0) || !(mission_timeout_s_ > 0.0)) {
    throw std::invalid_argument("mission_stub: service_time_s >= 0 and mission_timeout_s > 0");
  }

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
              "PLACEHOLDER G5: one fixed job to (%.2f, %.2f), no dispatch policy. "
              "Replace with capstone_mission.",
              goal_x_, goal_y_);
  if (!auto_start_) {
    RCLCPP_INFO(get_logger(), "auto_start=false: waiting for an external start (not implemented yet)");
  }
}

void MissionStubNode::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
  pose_ = Pose2D{message.pose.pose.position.x, message.pose.pose.position.y,
                 yaw_from_quaternion(message.pose.pose.orientation)};
  has_pose_ = true;
}

void MissionStubNode::tick() {
  if (state_ == State::Done) {
    return;
  }
  if (auto_start_ && !started_) {
    start_time_ = now();
    started_ = true;
    RCLCPP_INFO(get_logger(), "episode %s started", episode_id_.c_str());
  }
  if (started_ && (now() - start_time_).seconds() > mission_timeout_s_) {
    finish("TIMEOUT", "mission timeout");
    return;
  }

  switch (state_) {
    case State::Init:
      if (!auto_start_ || !has_pose_ || !path_client_->action_server_is_ready() ||
          !follow_client_->action_server_is_ready()) {
        return;
      }
      send_path_goal();
      state_ = State::Planning;
      break;
    case State::Planning:
    case State::Driving:
      break;
    case State::Service:
      if ((now() - service_start_).seconds() >= service_time_s_) {
        finish("SUCCESS", "job delivered");
      }
      break;
    case State::Done:
      break;
  }
}

void MissionStubNode::send_path_goal() {
  ComputePath::Goal goal;
  goal.goal.header.frame_id = map_frame_;
  goal.goal.header.stamp = now();
  goal.goal.pose.position.x = goal_x_;
  goal.goal.pose.position.y = goal_y_;
  goal.goal.pose.orientation = yaw_to_quaternion(goal_yaw_);
  goal.start.header.frame_id = map_frame_;
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
  RCLCPP_INFO(get_logger(), "requested path to (%.2f, %.2f)", goal_x_, goal_y_);
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
  RCLCPP_INFO(get_logger(), "requested FollowPath with %zu poses", path.poses.size());
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
  RCLCPP_INFO(get_logger(), "arrived at the station, servicing for %.1f s", service_time_s_);
}

void MissionStubNode::finish(const std::string &terminal_reason, const std::string &message) {
  if (finished_) {
    return;
  }
  finished_ = true;
  terminal_reason_ = terminal_reason;
  terminal_message_ = message;
  state_ = State::Done;
  write_episode();
  publish_status();
  RCLCPP_INFO(get_logger(), "episode %s finished: %s (%s)", episode_id_.c_str(),
              terminal_reason.c_str(), message.c_str());
}

void MissionStubNode::write_episode() {
  EpisodeRecord record;
  record.episode_id = episode_id_;
  record.config_id = config_id_;
  record.scenario_id = scenario_id_;
  record.terminal_reason = terminal_reason_;
  record.elapsed_sim_s = started_ ? (now() - start_time_).seconds() : 0.0;
  record.delivered = terminal_reason_ == "SUCCESS" ? 1 : 0;
  record.released = 1;
  record.on_time = 0;
  record.collision_count = 0;
  std::vector<Pose2D> path_points;
  path_points.reserve(path_msg_.poses.size());
  for (const auto &pose : path_msg_.poses) {
    path_points.push_back(Pose2D{pose.pose.position.x, pose.pose.position.y,
                                 yaw_from_quaternion(pose.pose.orientation)});
  }
  record.distance_m = path_length(path_points);

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
  status.message = std::string("state=") + state_name(state_) + " reason=" + terminal_reason_;
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

}  // namespace capstone_integration
