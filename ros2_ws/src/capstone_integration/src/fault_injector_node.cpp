#include "capstone_integration/fault_injector_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace capstone_integration {

namespace {

using namespace std::chrono_literals;

constexpr double kZeroSpeed = 0.01;

std::string number(double value) {
  if (value < 0.0) {
    return "NA";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

std::string sanitize(std::string value) {
  for (char &c : value) {
    if (c == ',' || c == '\n' || c == '\r') {
      c = ';';
    }
  }
  return value;
}

std::string topic_for(const std::string &fault) {
  if (fault == "sensor_dropout") {
    return "/fault/sensor_dropout";
  }
  if (fault == "controller_pause") {
    return "/fault/controller_pause";
  }
  if (fault == "controller_crash") {
    return "/fault/controller_crash";
  }
  throw std::invalid_argument(
      "fault_injector: fault must be sensor_dropout|controller_pause|controller_crash");
}

}  // namespace

FaultInjectorNode::FaultInjectorNode() : rclcpp::Node("fault_injector") {
  fault_ = declare_parameter<std::string>("fault", "sensor_dropout");
  run_id_ = declare_parameter<std::string>("run_id", "fault-1");
  output_dir_ = declare_parameter<std::string>("output_dir", "/workspace/results/faults");
  delay_s_ = declare_parameter<double>("delay_s", 0.5);
  duration_s_ = declare_parameter<double>("duration_s", 5.0);
  recovery_s_ = declare_parameter<double>("recovery_s", 3.0);
  motion_threshold_ = declare_parameter<double>("motion_threshold", 0.1);
  timeout_s_ = declare_parameter<double>("timeout_s", 60.0);

  if (!(delay_s_ >= 0.0) || !(duration_s_ > 0.0) || !(recovery_s_ >= 0.0) ||
      !(motion_threshold_ > 0.0) || !(timeout_s_ > 0.0)) {
    throw std::invalid_argument("fault_injector: durations must be positive");
  }

  const std::string topic = topic_for(fault_);
  fault_pub_ = create_publisher<std_msgs::msg::Bool>(topic, rclcpp::QoS(1).reliable());
  cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::TwistStamped &message) { on_cmd(message); });
  diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray &message) { on_diagnostics(message); });

  started_ = std::chrono::steady_clock::now();
  timer_ = create_wall_timer(50ms, [this]() { tick(); });
  RCLCPP_WARN(get_logger(), "fault injector armed: %s on %s -> %s", fault_.c_str(), topic.c_str(),
              output_dir_.c_str());
}

void FaultInjectorNode::on_cmd(const geometry_msgs::msg::TwistStamped &message) {
  if (std::isfinite(message.twist.linear.x)) {
    last_v_ = message.twist.linear.x;
  }
}

void FaultInjectorNode::on_diagnostics(
    const diagnostic_msgs::msg::DiagnosticArray &message) {
  if (phase_ != Phase::Injected || diag_latency_ >= 0.0) {
    return;
  }
  for (const auto &status : message.status) {
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      diag_latency_ = seconds_since(inject_time_);
      diag_reason_ = status.name + ": " + status.message;
      return;
    }
  }
}

double FaultInjectorNode::seconds_since(std::chrono::steady_clock::time_point point) const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - point).count();
}

void FaultInjectorNode::publish_fault(bool value) {
  std_msgs::msg::Bool message;
  message.data = value;
  fault_pub_->publish(message);
}

void FaultInjectorNode::inject() {
  inject_time_ = std::chrono::steady_clock::now();
  publish_fault(true);
  RCLCPP_WARN(get_logger(), "FAULT INJECTED: %s", fault_.c_str());
}

void FaultInjectorNode::tick() {
  if (finished_) {
    return;
  }
  if (phase_ == Phase::WaitMotion) {
    if (std::abs(last_v_) > motion_threshold_) {
      if (!have_motion_) {
        have_motion_ = true;
        phase_start_ = std::chrono::steady_clock::now();
      } else if (seconds_since(phase_start_) >= delay_s_) {
        inject();
        phase_ = Phase::Injected;
      }
    } else if (seconds_since(started_) > timeout_s_) {
      finish("no motion: /cmd_vel stayed zero");
    }
    return;
  }

  if (phase_ == Phase::Injected) {
    publish_fault(true);  // idempotent; covers a late subscription match
    if (zero_latency_ < 0.0 && std::abs(last_v_) <= kZeroSpeed) {
      zero_latency_ = seconds_since(inject_time_);
    }
    if (seconds_since(inject_time_) >= duration_s_) {
      if (fault_ != "controller_crash") {
        publish_fault(false);
      }
      phase_ = Phase::Recovering;
      phase_start_ = std::chrono::steady_clock::now();
    }
    return;
  }

  if (phase_ == Phase::Recovering) {
    if (fault_ != "controller_crash") {
      publish_fault(false);
    }
    if (std::abs(last_v_) > motion_threshold_) {
      recovered_ = true;
    }
    if (seconds_since(phase_start_) >= recovery_s_) {
      finish(recovered_ ? "recovered after the fault was released"
                        : "no recovery within the window");
    }
  }
}

void FaultInjectorNode::finish(const std::string &notes) {
  if (finished_) {
    return;
  }
  finished_ = true;
  phase_ = Phase::Done;
  write_row(notes);
  result_ = zero_latency_ >= 0.0 ? 0 : 3;
  RCLCPP_INFO(get_logger(), "fault %s: stop latency %s s, diagnostic latency %s s, recovered=%s",
              fault_.c_str(), number(zero_latency_).c_str(), number(diag_latency_).c_str(),
              recovered_ ? "true" : "false");
}

void FaultInjectorNode::write_row(const std::string &notes) {
  try {
    std::filesystem::create_directories(output_dir_);
    const std::filesystem::path file = std::filesystem::path(output_dir_) / "faults.csv";
    std::ofstream out(file, std::ios::app);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", file.c_str());
      return;
    }
    if (out.tellp() == 0) {
      out << "run_id,fault,stop_latency_s,diag_latency_s,diag_reason,recovered,notes\n";
    }
    out << sanitize(run_id_) << ',' << sanitize(fault_) << ',' << number(zero_latency_) << ','
        << number(diag_latency_) << ',' << sanitize(diag_reason_) << ','
        << (recovered_ ? "true" : "false") << ',' << sanitize(notes) << '\n';
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "cannot write the fault row: %s", error.what());
  }
}

}  // namespace capstone_integration
