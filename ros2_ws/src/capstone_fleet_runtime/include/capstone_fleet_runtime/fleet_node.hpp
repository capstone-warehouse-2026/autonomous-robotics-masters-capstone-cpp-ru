#pragma once

#include <chrono>
#include <fstream>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "capstone_fleet_runtime/robot_navigator.hpp"
#include "capstone_fleet_runtime/scene.hpp"
#include "capstone_fleet_runtime/fleet_jobs.hpp"

namespace capstone_fleet_runtime {

class FleetNode : public rclcpp::Node {
 public:
  explicit FleetNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void request_shutdown();
  bool finished() const { return finished_; }
  int exit_code() const { return exit_code_; }

 private:
  using Clock = std::chrono::steady_clock;
  enum class State { WaitingReady, Idle, Planning, Executing, Servicing, Stopping, Completed, Cancelled, Fault };
  enum class TaskPhase { Idle, ToPickup, PickupService, ToDropoff, DropoffService };
  struct Robot {
    RobotSpec spec;
    capstone_fleet::Cell cell;
    std::unique_ptr<RobotNavigator> navigation;
    TaskPhase phase{TaskPhase::Idle};
    std::string job;
    double service_until{0};
  };

  void read_parameters();
  void create_interfaces();
  void on_map(const nav_msgs::msg::OccupancyGrid &message);
  void on_tick();
  void wait_ready();
  void release_jobs();
  void assign_and_plan();
  void assign_jobs();
  void start_planning();
  void poll_planning();
  void validate_joint_plan(const capstone_fleet::PlanResult &result) const;
  void start_round();
  void poll_round();
  void start_services();
  void poll_services();
  void begin_stop(State terminal, const std::string &reason);
  void poll_stopping();
  void finish(State terminal, const std::string &reason, bool stopped);
  bool all_stopped() const;
  double elapsed() const;
  std::string status_name() const;
  void publish_status();
  void event(const std::string &name, const std::string &job = "",
             const std::string &robot = "", const std::string &detail = "");
  void save_assignment(const std::vector<std::size_t> &idle,
                       const std::vector<const capstone_mission::Job *> &jobs,
                       const std::vector<std::vector<int>> &costs,
                       const capstone_fleet::AssignmentResult &assignment);
  void save_plan(const capstone_fleet::PlanResult &result);

  Scene scene_;
  FleetJobQueue jobs_;
  capstone_fleet::Grid grid_;
  capstone_fleet::PlanOptions plan_options_;
  std::vector<Robot> robots_;
  std::string manifest_path_, output_directory_, method_, terminal_reason_;
  State state_{State::WaitingReady}, requested_terminal_{State::Fault};
  double footprint_radius_{0.4}, startup_timeout_{60}, stop_timeout_{6}, edge_timeout_{30};
  double epoch_{0}, previous_clock_{0};
  bool map_ready_{false}, finished_{false}, started_{false};
  int exit_code_{1}, plan_id_{0}, assignment_id_{0};
  std::size_t step_{0};
  Clock::time_point started_wall_{Clock::now()}, last_clock_progress_{Clock::now()}, stopping_since_{};
  std::optional<Clock::time_point> ready_since_;
  std::future<capstone_fleet::PlanResult> pending_plan_;
  std::vector<capstone_fleet::Cell> plan_starts_, plan_goals_;
  capstone_fleet::Paths joint_paths_;
  std::ofstream events_, plans_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace capstone_fleet_runtime
