#pragma once

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "capstone_mission/dispatch.hpp"
#include "capstone_mission/navigation_client.hpp"

namespace capstone_mission {

class MissionNode : public rclcpp::Node {
 public:
  explicit MissionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  const std::string &method() const { return method_; }
  void request_shutdown();
  bool shutdown_ready() const;

 private:
  enum class State { Idle, Running, Cancelling, Completed, Cancelled, Fault };
  using Clock = std::chrono::steady_clock;
  using Trigger = std_srvs::srv::Trigger;

  void read_parameters();
  void load_configuration();
  void create_ros_interfaces();
  void create_behavior_tree();
  void start_episode();
  void reset_episode(bool activate);
  void on_timer();
  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray &message);
  void begin_cancel(const std::string &reason, bool reset_after = false);
  void finish_cancel();
  bool inputs_ready() const;
  bool fresh_localization() const;
  bool safety_inputs_fresh() const;
  double episode_time() const;
  void release_jobs();
  DispatchContext dispatch_context() const;
  geometry_msgs::msg::PoseStamped target_pose(const std::string &station_id) const;
  BT::NodeStatus wait_ready();
  BT::NodeStatus select_job();
  BT::NodeStatus navigate(bool pickup);
  BT::NodeStatus service(bool pickup);
  BT::NodeStatus complete_job();
  void fail_job(const std::string &reason);
  void record_event(const std::string &event, const std::string &reason = "",
                    const std::string &target = "");
  void publish_status();
  std::string state_name() const;

  std::string method_;
  std::string manifest_path_, model_path_, tree_path_, event_log_path_;
  std::string model_error_, active_job_, navigation_target_, cancel_reason_;
  std::string episode_id_, safety_reason_;
  bool autostart_{true}, require_safety_{true}, congestion_feature_{true};
  bool reset_after_cancel_{false}, service_started_{false}, shutdown_requested_{false};
  bool configured_{false}, clock_seen_{false};
  int attempts_{0}, navigation_failures_{0}, fallback_count_{0}, episode_sequence_{0};
  double clock_timeout_{2.0}, readiness_timeout_{15.0}, wait_until_{0.0};
  double retry_at_{0.0}, service_until_{0.0}, epoch_{0.0}, last_clock_{0.0};
  NavigationClient::Limits navigation_limits_;
  geometry_msgs::msg::PoseWithCovarianceStamped localization_;
  Clock::time_point localization_received_{}, scan_received_{}, grid_received_{};
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME}, grid_stamp_{0, 0, RCL_ROS_TIME};
  Scenario scenario_;
  JobQueue queue_;
  std::unique_ptr<DqnModel> model_;
  std::unique_ptr<NavigationClient> navigation_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  State state_{State::Idle};
  std::ofstream events_;
  Clock::time_point last_clock_progress_{Clock::now()}, episode_started_{Clock::now()};
  Clock::time_point last_status_{}, safety_received_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, cancel_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};

}  // namespace capstone_mission
