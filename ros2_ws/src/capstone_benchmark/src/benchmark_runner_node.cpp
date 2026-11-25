#include "capstone_benchmark/benchmark_runner_node.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace capstone_benchmark {
BenchmarkRunnerNode::BenchmarkRunnerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("benchmark_runner", options) {
  method_ = declare_parameter<std::string>("method", "classical");
  if (method_ != "classical" && method_ != "learned") {
    throw std::invalid_argument("method must be classical or learned");
  }
  enabled_ = declare_parameter("run_episode", false);
  output_directory_ = declare_parameter<std::string>("output_directory", "/tmp/g5-episode");
  reset_service_ = declare_parameter<std::string>("simulation_reset_service", "debug/reset");
  episode_id_ = declare_parameter<std::string>("episode_id", "debug_episode");
  commit_ = declare_parameter<std::string>("code_commit", "unrecorded");
  horizon_ = declare_parameter("horizon", 300.0);
  startup_timeout_ = declare_parameter("startup_timeout", 10.0);
  clock_timeout_ = declare_parameter("clock_timeout", 1.0);
  cancel_timeout_ = declare_parameter("cancel_timeout", 3.0);
  service_settle_time_ = declare_parameter("service_settle_time", 0.0);
  if (!std::isfinite(service_settle_time_) || service_settle_time_ < 0 ||
      service_settle_time_ >= startup_timeout_) {
    throw std::invalid_argument("service_settle_time must be nonnegative and below startup_timeout");
  }
  for (double value : {horizon_, startup_timeout_, clock_timeout_, cancel_timeout_}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("timeouts must be positive");
  }
  if (!enabled_) return;
  if (std::filesystem::exists(std::filesystem::path(output_directory_) / "episode.yaml")) {
    throw std::invalid_argument("episode output exists; choose a new directory");
  }
  create_interfaces();
  stage_ = Stage::WaitingServices;
  stage_since_ = Clock::now();
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { on_tick(); });
}

void BenchmarkRunnerNode::create_interfaces() {
  simulation_reset_ = create_client<Trigger>(reset_service_);
  mission_reset_ = create_client<Trigger>("mission/reset");
  mission_start_ = create_client<Trigger>("mission/start");
  mission_cancel_ = create_client<Trigger>("mission/cancel");
  clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::SensorDataQoS(), [this](const rosgraph_msgs::msg::Clock &m) { on_clock(m); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "state/odom", rclcpp::QoS(5).reliable(), [this](const nav_msgs::msg::Odometry &m) { on_odom(m); });
  status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "mission/status", 10, [this](const diagnostic_msgs::msg::DiagnosticArray &m) { on_status(m); });
}

double BenchmarkRunnerNode::age(Clock::time_point point) const {
  return std::chrono::duration<double>(Clock::now() - point).count();
}

void BenchmarkRunnerNode::request(const rclcpp::Client<Trigger>::SharedPtr &client,
                                  Stage pending, Stage next) {
  stage_ = pending;
  stage_since_ = Clock::now();
  client->async_send_request(std::make_shared<Trigger::Request>(),
    [this, pending, next](rclcpp::Client<Trigger>::SharedFuture future) {
      if (stage_ != pending) return;
      const auto response = future.get();
      if (!response->success) { stop("service_failed: " + response->message); return; }
      stage_ = next;
      stage_since_ = Clock::now();
      if (next == Stage::WaitingInputs) { have_odom_ = false; have_clock_ = false; clock_updates_ = 0; }
    });
}

void BenchmarkRunnerNode::on_tick() {
  if (finished_) return;
  if (stage_ == Stage::Cancelling) {
    if (age(stage_since_) > cancel_timeout_) finish(terminal_reason_ + ":cancel_timeout", false);
    return;
  }
  if (stage_ == Stage::Running) {
    if (age(last_clock_) > clock_timeout_) stop("frozen_clock");
    else if (age(last_odom_) > clock_timeout_) stop("missing_odom");
    else if (simulation_time_ - start_time_ >= horizon_) stop("timeout");
    return;
  }
  if (age(stage_since_) > startup_timeout_) { stop("startup_timeout"); return; }
  if (stage_ == Stage::WaitingServices) {
    const bool all_ready = simulation_reset_->service_is_ready() && mission_reset_->service_is_ready() &&
                           mission_start_->service_is_ready() && mission_cancel_->service_is_ready();
    if (!all_ready) {
      services_ready_since_.reset();
      return;
    }
    if (!services_ready_since_) services_ready_since_ = Clock::now();
    // FastDDS может объявить request-канал готовым раньше discovery ответного reader.
    // Ожидание ограничено startup_timeout и не блокирует callbacks или таймеры.
    if (age(*services_ready_since_) >= service_settle_time_) {
      request(simulation_reset_, Stage::ResetSimulation, Stage::ResetMission);
    }
  } else if (stage_ == Stage::ResetMission) {
    request(mission_reset_, Stage::WaitingMissionReset, Stage::WaitingInputs);
  } else if (stage_ == Stage::WaitingInputs && clock_updates_ >= 2 && have_odom_ &&
             age(last_clock_) < clock_timeout_ && age(last_odom_) < 0.3) {
    start_time_ = simulation_time_;
    started_ = true;
    distance_ = 0;
    // Running устанавливается до ответа: короткая миссия может завершиться сразу.
    request(mission_start_, Stage::Running, Stage::Running);
  }
}

void BenchmarkRunnerNode::on_clock(const rosgraph_msgs::msg::Clock &message) {
  const double time = rclcpp::Time(message.clock).seconds();
  if (!have_clock_ || time > simulation_time_) { last_clock_ = Clock::now(); ++clock_updates_; }
  if (stage_ == Stage::Running && time < simulation_time_) stop("unexpected_time_reset");
  simulation_time_ = time;
  have_clock_ = true;
}

void BenchmarkRunnerNode::on_odom(const nav_msgs::msg::Odometry &message) {
  const double x = message.pose.pose.position.x, y = message.pose.pose.position.y;
  if (!std::isfinite(x) || !std::isfinite(y) || message.header.frame_id != "odom" ||
      message.child_frame_id != "base_link") return;
  const double stamp = rclcpp::Time(message.header.stamp).seconds();
  if (std::abs(simulation_time_ - stamp) > 0.3) return;
  if (stage_ == Stage::Running && have_odom_) distance_ += std::hypot(x - x_, y - y_);
  x_ = x; y_ = y;
  have_odom_ = true;
  last_odom_ = Clock::now();
}

void BenchmarkRunnerNode::on_status(const diagnostic_msgs::msg::DiagnosticArray &message) {
  const double stamp = rclcpp::Time(message.header.stamp).seconds();
  if (!started_ || stamp < start_time_ || std::abs(simulation_time_ - stamp) > 0.3) return;
  for (const auto &status : message.status) {
    if (status.name != "capstone_mission/mission") continue;
    std::string episode;
    for (const auto &value : status.values) if (value.key == "episode_id") episode = value.value;
    // Начальное running связывает runner с новым эпизодом; старые terminal игнорируются.
    if (status.message == "running" && active_episode_.empty()) active_episode_ = episode;
    if (episode.empty() || episode != active_episode_) continue;
    if (stage_ == Stage::Running && status.message == "completed") finish("completed", true);
    else if (stage_ == Stage::Running && status.message == "fault") stop("mission_fault");
    else if (stage_ == Stage::Running && status.message == "cancelled") finish("mission_cancelled", false);
    else if (stage_ == Stage::Cancelling &&
             (status.message == "cancelled" || status.message == "fault")) {
      finish(terminal_reason_, false);
    }
  }
}

void BenchmarkRunnerNode::stop(const std::string &reason) {
  if (stage_ == Stage::Cancelling || finished_) return;
  terminal_reason_ = reason;
  stage_ = Stage::Cancelling;
  stage_since_ = Clock::now();
  if (!mission_cancel_->service_is_ready()) { finish(reason + ":cancel_unavailable", false); return; }
  // Ответ Trigger подтверждает принятие команды; окончание ждём по статусу миссии.
  mission_cancel_->async_send_request(std::make_shared<Trigger::Request>());
}

void BenchmarkRunnerNode::finish(const std::string &reason, bool success) {
  if (finished_) return;
  std::filesystem::create_directories(output_directory_);
  YAML::Node result;
  result["schema_version"] = 1;
  result["episode_id"] = active_episode_.empty() ? episode_id_ : active_episode_;
  result["method"] = method_;
  result["code_commit"] = commit_;
  result["terminal_reason"] = reason;
  result["elapsed_sim_s"] = started_ ? std::max(0.0, simulation_time_ - start_time_) : 0.0;
  result["distance_m"] = distance_;
  result["simulation_reset_service"] = reset_service_;
  result["scope"] = reset_service_ == "debug/reset" ? "debug_kinematic_actions" : "external_navigation";
  std::ofstream output(std::filesystem::path(output_directory_) / "episode.yaml");
  output << result << '\n';
  if (!output) throw std::runtime_error("cannot write episode manifest");
  exit_code_ = success ? 0 : 1;
  finished_ = true;
  stage_ = Stage::Finished;
  RCLCPP_INFO(get_logger(), "episode finished: %s", reason.c_str());
}
}  // namespace capstone_benchmark
