#pragma once
#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace capstone_benchmark {
// reset simulator → reset mission → ready inputs → start → terminal/cancel → manifest.
class BenchmarkRunnerNode : public rclcpp::Node {
 public:
  explicit BenchmarkRunnerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  const std::string &method() const { return method_; }
  bool finished() const { return finished_; }
  int exit_code() const { return exit_code_; }
 private:
  enum class Stage { Disabled, WaitingServices, ResetSimulation, ResetMission, WaitingMissionReset,
                     WaitingInputs, Running, Cancelling, Finished };
  using Trigger = std_srvs::srv::Trigger;
  using Clock = std::chrono::steady_clock;
  void create_interfaces();
  void on_tick();
  void on_clock(const rosgraph_msgs::msg::Clock &message);
  void on_odom(const nav_msgs::msg::Odometry &message);
  void on_status(const diagnostic_msgs::msg::DiagnosticArray &message);
  void request(const rclcpp::Client<Trigger>::SharedPtr &client, Stage pending, Stage next);
  void stop(const std::string &reason);
  void finish(const std::string &reason, bool success);
  double age(Clock::time_point point) const;
  std::string method_, output_directory_, reset_service_, episode_id_, commit_;
  double horizon_{300}, startup_timeout_{10}, clock_timeout_{1}, cancel_timeout_{3};
  double service_settle_time_{0};
  std::optional<Clock::time_point> services_ready_since_;
  bool enabled_{false}, finished_{false}, have_clock_{false}, have_odom_{false}, started_{false};
  int exit_code_{0};
  int clock_updates_{0};
  std::string active_episode_;
  Stage stage_{Stage::Disabled};
  Clock::time_point stage_since_, last_clock_, last_odom_;
  double simulation_time_{0}, start_time_{0}, distance_{0}, x_{0}, y_{0};
  std::string terminal_reason_;
  rclcpp::Client<Trigger>::SharedPtr simulation_reset_, mission_reset_, mission_start_, mission_cancel_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace capstone_benchmark
