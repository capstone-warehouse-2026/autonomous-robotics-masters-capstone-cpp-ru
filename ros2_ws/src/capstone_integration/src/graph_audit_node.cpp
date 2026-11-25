#include "capstone_integration/graph_audit_node.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <utility>

namespace capstone_integration {

namespace {

using namespace std::chrono_literals;

tf2::TimePoint zero_time() { return tf2::TimePointZero; }

}  // namespace

GraphAuditNode::GraphAuditNode() : rclcpp::Node("graph_audit") {
  settle_s_ = declare_parameter<double>("settle_s", 2.0);
  window_s_ = declare_parameter<double>("window_s", 5.0);
  fail_on_ = declare_parameter<std::string>("fail_on", "error");
  json_path_ = declare_parameter<std::string>("json_path", "");
  measured_run_ = declare_parameter<bool>("measured_run", false);
  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);

  if (!(settle_s_ >= 0.0) || !(window_s_ > 0.0)) {
    throw std::invalid_argument("graph_audit: settle_s >= 0 and window_s > 0");
  }
  if (fail_on_ != "never" && fail_on_ != "warn" && fail_on_ != "error") {
    throw std::invalid_argument("graph_audit: fail_on must be never|warn|error");
  }

  const auto sensor_qos = rclcpp::SensorDataQoS();
  for (const auto &contract : contracts()) {
    if (contract.name == "/clock") {
      auto subscription = create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::QoS(10),
          [this](const rosgraph_msgs::msg::Clock &message) { on_clock(message); });
      subscriptions_.push_back(subscription);
    } else if (contract.name == "/sensors/scan") {
      subscribe_topic<sensor_msgs::msg::LaserScan>(contract.name, sensor_qos);
    } else if (contract.name == "/sensors/rgb" || contract.name == "/sensors/depth") {
      subscribe_topic<sensor_msgs::msg::Image>(contract.name, sensor_qos);
    } else if (contract.name == "/sensors/camera_info") {
      subscribe_topic<sensor_msgs::msg::CameraInfo>(contract.name, sensor_qos);
    } else if (contract.name == "/sensors/imu") {
      subscribe_topic<sensor_msgs::msg::Imu>(contract.name, sensor_qos);
    } else if (contract.name == "/sensors/wheel_odom" || contract.name == "/state/odom") {
      subscribe_topic<nav_msgs::msg::Odometry>(contract.name, sensor_qos);
    } else if (contract.name == "/localization/pose") {
      subscribe_topic<geometry_msgs::msg::PoseWithCovarianceStamped>(contract.name, sensor_qos);
    } else if (contract.name == "/map") {
      subscribe_topic<nav_msgs::msg::OccupancyGrid>(
          contract.name, rclcpp::QoS(1).reliable().transient_local());
    } else if (contract.name == "/perception/obstacles") {
      subscribe_topic<nav_msgs::msg::OccupancyGrid>(contract.name, sensor_qos);
    } else if (contract.name == "/control/cmd_vel_raw" || contract.name == "/cmd_vel") {
      subscribe_topic<geometry_msgs::msg::TwistStamped>(contract.name, sensor_qos);
    } else if (contract.name == "/mission/status" || contract.name == "/diagnostics") {
      subscribe_topic<diagnostic_msgs::msg::DiagnosticArray>(contract.name, sensor_qos);
    }
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());

  phase_start_ = std::chrono::steady_clock::now();
  timer_ = create_wall_timer(50ms, [this]() { tick(); });
  RCLCPP_INFO(get_logger(),
              "contract audit: settling %.1f s, sampling %.1f s, measured_run=%s",
              settle_s_, window_s_, measured_run_ ? "true" : "false");
}

void GraphAuditNode::on_clock(const rosgraph_msgs::msg::Clock &message) {
  if (!sampling_) {
    return;
  }
  ++clock_count_;
  const double stamp = static_cast<double>(message.clock.sec) +
                       static_cast<double>(message.clock.nanosec) * 1e-9;
  if (clock_count_ == 1) {
    clock_first_s_ = stamp;
  } else if (stamp < clock_last_s_) {
    clock_monotonic_ = false;
  }
  clock_last_s_ = stamp;
}

void GraphAuditNode::count(const std::string &topic) {
  if (sampling_) {
    ++counters_[topic];
  }
}

void GraphAuditNode::record_frame(const std::string &topic, const std::string &frame) {
  if (sampling_) {
    frames_[topic] = frame;
  }
}

void GraphAuditNode::tick() {
  if (finished_) {
    return;
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - phase_start_).count();
  if (phase_ == Phase::Settle && elapsed >= settle_s_) {
    begin_sampling();
  } else if (phase_ == Phase::Sampling && elapsed >= window_s_) {
    finish();
  }
}

void GraphAuditNode::begin_sampling() {
  sampling_ = true;
  phase_ = Phase::Sampling;
  phase_start_ = std::chrono::steady_clock::now();
  counters_.clear();
  frames_.clear();
  clock_count_ = 0;
  clock_first_s_ = 0.0;
  clock_last_s_ = -1.0;
  clock_monotonic_ = true;
  RCLCPP_INFO(get_logger(), "sampling the contract for %.1f s", window_s_);
}

void GraphAuditNode::finish() {
  if (finished_) {
    return;
  }
  timer_->cancel();
  sampling_ = false;

  std::map<std::string, std::string> type_by_topic;
  for (const auto &entry : get_topic_names_and_types()) {
    if (!entry.second.empty()) {
      type_by_topic[entry.first] = entry.second.front();
    }
  }

  std::vector<Row> rows;
  rows.push_back(evaluate_clock(clock_count_, clock_first_s_, clock_last_s_, clock_monotonic_,
                                static_cast<int>(count_publishers("/clock"))));

  for (const auto &contract : contracts()) {
    if (contract.name == "/clock") {
      continue;
    }
    TopicSample sample;
    sample.contract = contract;
    const auto topic = type_by_topic.find(contract.name);
    sample.exists = topic != type_by_topic.end();
    if (sample.exists) {
      sample.actual_type = topic->second;
    }
    sample.publishers = static_cast<int>(count_publishers(contract.name));
    sample.messages = counters_[contract.name];
    sample.measured_hz = static_cast<double>(sample.messages) / window_s_;
    const auto frame = frames_.find(contract.name);
    if (frame != frames_.end()) {
      sample.frame = frame->second;
    }
    rows.push_back(evaluate_topic(sample));
  }

  std::set<std::string> service_names;
  for (const auto &entry : get_service_names_and_types()) {
    service_names.insert(entry.first);
  }
  const auto has_action = [&service_names](const std::string &name) {
    return service_names.count(name + "/_action/send_goal") > 0;
  };
  rows.push_back(evaluate_action("/planning/compute_path", has_action("/planning/compute_path")));
  rows.push_back(evaluate_action("/control/follow_path", has_action("/control/follow_path")));

  const bool map_to_base = tf_buffer_->canTransform("map", "base_link", zero_time());
  const bool map_to_odom = tf_buffer_->canTransform("map", "odom", zero_time());
  const bool odom_to_base = tf_buffer_->canTransform("odom", "base_link", zero_time());
  rows.push_back(evaluate_tf(map_to_base, map_to_odom, odom_to_base));

  rows.push_back(evaluate_placeholders(get_node_names(), measured_run_));

  std::vector<std::string> ground_truth_subscribers;
  for (const auto &info : get_subscriptions_info_by_topic("/evaluation/ground_truth")) {
    ground_truth_subscribers.push_back(info.node_namespace() + "/" + info.node_name());
  }
  rows.push_back(evaluate_ground_truth(ground_truth_subscribers));

  int extras = 0;
  for (const auto &entry : type_by_topic) {
    const bool in_contract =
        std::any_of(contracts().begin(), contracts().end(),
                    [&entry](const Contract &contract) { return contract.name == entry.first; });
    if (!in_contract && !is_allowed_extra_topic(entry.first) && extras < 10) {
      rows.push_back(Row{entry.first, Verdict::Info, "topic outside the contract"});
      ++extras;
    }
  }

  int failures = 0;
  int warnings = 0;
  for (const auto &row : rows) {
    failures += row.verdict == Verdict::Fail ? 1 : 0;
    warnings += row.verdict == Verdict::Warn ? 1 : 0;
  }

  std::cout << "\n" << format_table(rows) << "\n";
  std::cout << "summary: " << failures << " FAIL, " << warnings << " WARN, "
            << static_cast<int>(rows.size()) - failures - warnings << " other; window "
            << window_s_ << " s" << std::endl;

  if (!json_path_.empty()) {
    std::ofstream out(json_path_);
    if (out) {
      out << to_json(rows, window_s_, measured_run_);
      RCLCPP_INFO(get_logger(), "audit written to %s", json_path_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "cannot write %s", json_path_.c_str());
    }
  }

  if (publish_diagnostics_) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    for (const auto &row : rows) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "graph_audit" + row.name;
      status.hardware_id = "capstone_integration";
      status.level = row.verdict == Verdict::Fail ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                       : row.verdict == Verdict::Warn
                           ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                           : diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = verdict_name(row.verdict) + ": " + row.detail;
      array.status.push_back(status);
    }
    diagnostics_pub_->publish(array);
  }

  exit_code_ = exit_code(rows, fail_on_);
  finished_ = true;
}

}  // namespace capstone_integration
