#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace capstone_mission {

// Запрос пути → ответ планировщика → FollowPath → result → подтверждение остановки.
// Отмена принятого запроса не считается конечным результатом action.
class NavigationClient {
 public:
  enum class State { Idle, Planning, Following, Stopping, Succeeded, Failed };
  struct Limits {
    double planning_timeout{0.2};
    double navigation_timeout{120.0};
    double stop_timeout{2.0};
    double stop_confirmation_time{0.2};
    double input_timeout{0.3};
  };
  using Event = std::function<void(const std::string &, const std::string &)>;

  NavigationClient(rclcpp::Node &node, Limits limits, Event event);
  bool ready() const;
  void start(const geometry_msgs::msg::PoseStamped &target);
  void cancel(const std::string &reason);
  void poll();
  void clear();
  State state() const { return state_; }
  bool stopped() const;
  bool safe_to_retry() const { return stop_confirmed_; }
  const std::string &reason() const { return reason_; }
  const nav_msgs::msg::Odometry &odometry() const { return odometry_; }
  bool fresh_odometry() const;
  double distance_to_goal() const { return distance_to_goal_; }

 private:
  using Plan = nav2_msgs::action::ComputePathToPose;
  using Follow = nav2_msgs::action::FollowPath;
  using PlanHandle = rclcpp_action::ClientGoalHandle<Plan>;
  using FollowHandle = rclcpp_action::ClientGoalHandle<Follow>;
  using Clock = std::chrono::steady_clock;

  void on_odometry(const nav_msgs::msg::Odometry &message);
  void process_pending_odometry();
  void accept_odometry(const nav_msgs::msg::Odometry &message, Clock::time_point received_at);
  void on_plan_response(std::uint64_t generation, const PlanHandle::SharedPtr &handle);
  void on_plan_result(std::uint64_t generation, const PlanHandle::WrappedResult &result);
  void send_follow(const nav_msgs::msg::Path &path);
  void on_follow_response(std::uint64_t generation, const FollowHandle::SharedPtr &handle);
  void on_follow_result(std::uint64_t generation, const FollowHandle::WrappedResult &result);
  void begin_stop(bool success, const std::string &reason);
  void cancel_owned_goals();
  bool valid_path(const nav_msgs::msg::Path &path) const;

  rclcpp::Node &node_;
  Limits limits_;
  Event event_;
  rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Client<Follow>::SharedPtr controller_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  PlanHandle::SharedPtr plan_handle_;
  FollowHandle::SharedPtr follow_handle_;
  nav_msgs::msg::Odometry odometry_;
  struct PendingOdometry {
    nav_msgs::msg::Odometry message;
    Clock::time_point received_at;
  };
  std::optional<PendingOdometry> pending_odometry_;
  std::optional<double> previous_clock_;
  struct QuietInterval { double stamp; Clock::time_point received_at; };
  std::optional<QuietInterval> quiet_interval_;
  double stop_requested_stamp_{0.0};
  State state_{State::Idle};
  std::uint64_t generation_{0};
  bool plan_pending_{false};
  bool follow_pending_{false};
  bool plan_terminal_{true};
  bool follow_terminal_{true};
  bool cancel_sent_plan_{false};
  bool cancel_sent_follow_{false};
  bool success_requested_{false};
  bool stop_confirmed_{false};
  bool odom_received_{false};
  std::string reason_;
  Clock::time_point stage_started_{};
  Clock::time_point odom_received_at_{};
  Clock::time_point stop_started_{};
  double distance_to_goal_{0.0};
};

}  // namespace capstone_mission
