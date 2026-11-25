#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace capstone_integration {

// Injects one fault while the robot is moving and measures the reaction (docs/INTERFACES.md,
// fault table): how long /cmd_vel stays non-zero, when the first WARN/ERROR diagnostic appears,
// and whether the system recovers after the fault is released.
class FaultInjectorNode : public rclcpp::Node {
 public:
  FaultInjectorNode();

  bool finished() const { return finished_; }
  int result() const { return result_; }

 private:
  enum class Phase { WaitMotion, Injected, Recovering, Done };

  void tick();
  void on_cmd(const geometry_msgs::msg::TwistStamped &message);
  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray &message);
  void inject();
  void publish_fault(bool value);
  void finish(const std::string &notes);
  void write_row(const std::string &notes);
  double seconds_since(std::chrono::steady_clock::time_point point) const;

  std::string fault_;
  std::string run_id_;
  std::string output_dir_;
  double delay_s_{0.5};
  double duration_s_{5.0};
  double recovery_s_{3.0};
  double motion_threshold_{0.1};
  double timeout_s_{60.0};

  Phase phase_{Phase::WaitMotion};
  std::chrono::steady_clock::time_point started_{};
  std::chrono::steady_clock::time_point phase_start_{};
  std::chrono::steady_clock::time_point inject_time_{};
  bool have_motion_{false};
  bool recovered_{false};
  bool finished_{false};
  int result_{0};
  double last_v_{0.0};
  double zero_latency_{-1.0};
  double diag_latency_{-1.0};
  std::string diag_reason_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace capstone_integration
