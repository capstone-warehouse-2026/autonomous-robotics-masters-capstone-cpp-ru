#pragma once

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "capstone_integration/kinematics.hpp"

namespace capstone_integration {

// PLACEHOLDER for G5. One job: plan to the goal station, follow the path, stop there for the
// service time, append one row to the episode CSV. It proves the G3/G4 action contract and the
// result format; it has no Behavior Tree and no dispatch policy.
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
  enum class State { Init, Planning, Driving, Service, Done };

  void tick();
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &message);
  void send_path_goal();
  void send_follow_goal(const nav_msgs::msg::Path &path);
  void on_path_result(const PathWrappedResult &result);
  void on_follow_result(const FollowWrappedResult &result);
  void finish(const std::string &terminal_reason, const std::string &message);
  void publish_status();
  void write_episode();
  static const char *state_name(State state);

  std::string map_frame_;
  double goal_x_{-8.8};
  double goal_y_{7.5};
  double goal_yaw_{1.5707963267948966};
  double service_time_s_{2.0};
  double mission_timeout_s_{300.0};
  std::string results_dir_{"/workspace/results"};
  std::string episode_id_{"stub-001"};
  std::string config_id_{"SKELETON"};
  std::string scenario_id_{"S1"};
  bool auto_start_{true};

  State state_{State::Init};
  bool started_{false};
  bool finished_{false};
  bool has_pose_{false};
  Pose2D pose_;
  nav_msgs::msg::Path path_msg_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time service_start_{0, 0, RCL_ROS_TIME};
  std::string terminal_reason_{"ABORTED"};
  std::string terminal_message_{"mission not finished"};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  PathClient::SharedPtr path_client_;
  FollowClient::SharedPtr follow_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace capstone_integration
