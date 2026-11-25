#pragma once

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "capstone_integration/jobs.hpp"
#include "capstone_integration/kinematics.hpp"

namespace capstone_integration {

// PLACEHOLDER for G5. Sequential delivery of the manifest jobs with the classic dispatch rule
// (earliest due date), capacity one, pick-up and drop-off serviced by a 2 s stop. There is no
// Behavior Tree, no preemption and no recovery: it exists to exercise the G3/G4 action contract and
// to produce honest episode rows.
class MissionStubNode : public rclcpp::Node {
 public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using PathClient = rclcpp_action::Client<ComputePath>;
  using FollowClient = rclcpp_action::Client<FollowPath>;
  using PathWrappedResult = PathClient::WrappedResult;
  using FollowWrappedResult = FollowClient::WrappedResult;

  explicit MissionStubNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  enum class State { Init, Idle, Planning, Driving, Service, Retry, Done };
  enum class Leg { Pickup, Dropoff };

  void tick();
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message);
  void release_jobs(double elapsed_s);
  void start_next_job();
  void send_path_goal(const Pose2D &target);
  void send_follow_goal(const nav_msgs::msg::Path &path);
  void on_path_result(const PathWrappedResult &result);
  void on_follow_result(const FollowWrappedResult &result);
  void complete_service();
  void finish(const std::string &terminal_reason, const std::string &message);
  void publish_status();
  void write_episode();
  std::vector<Pose2D> path_points() const;
  static const char *state_name(State state);
  static const char *leg_name(Leg leg);

  double service_time_s_{2.0};
  double mission_timeout_s_{300.0};
  double dispatch_delay_s_{2.0};
  int max_path_retries_{5};
  std::string results_dir_{"/workspace/results"};
  std::string episode_id_{"stub-001"};
  std::string config_id_{"SKELETON"};
  std::string scenario_id_{"S1"};
  std::string jobs_file_;
  bool auto_start_{true};

  std::vector<Job> jobs_;
  std::vector<bool> released_;
  std::vector<bool> delivered_;
  std::size_t active_job_{0};
  bool has_active_{false};
  Leg leg_{Leg::Pickup};
  Pose2D pose_;
  Pose2D target_;
  std::string active_target_name_;

  State state_{State::Init};
  bool started_{false};
  bool finished_{false};
  bool has_pose_{false};
  nav_msgs::msg::Path path_msg_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time service_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time retry_after_{0, 0, RCL_ROS_TIME};
  int path_retries_{0};
  std::string terminal_reason_{"TIMEOUT"};
  std::string terminal_message_{"mission not finished"};
  int on_time_{0};
  double distance_m_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  PathClient::SharedPtr path_client_;
  FollowClient::SharedPtr follow_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace capstone_integration
