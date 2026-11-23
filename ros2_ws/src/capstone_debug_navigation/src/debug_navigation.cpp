// Отдельная кинематическая среда для проверки обмена G5 с G3/G4.
// Это не реализация локализации, безопасной навигации или физики Webots.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace capstone_debug_navigation {
using Plan = nav2_msgs::action::ComputePathToPose;
using Follow = nav2_msgs::action::FollowPath;
using PlanHandle = rclcpp_action::ServerGoalHandle<Plan>;
using FollowHandle = rclcpp_action::ServerGoalHandle<Follow>;
using Steady = std::chrono::steady_clock;

class DebugNavigation : public rclcpp::Node {
 public:
  DebugNavigation() : Node("debug_navigation") {
    read_parameters();
    create_ros_interfaces();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { on_tick(); });
    RCLCPP_WARN(get_logger(), "G5 debug kinematic navigation, not a full robot simulation");
  }

 private:
  void read_parameters() {
    speed_ = declare_parameter("speed", 1.0);
    time_scale_ = declare_parameter("time_scale", 5.0);
    declare_parameter("planning_delay", 0.02);
    declare_parameter("blocked_route", false);
    declare_parameter("controller_failure", false);
    declare_parameter("failure_to_stop", false);
    declare_parameter("freeze_clock", false);
    declare_parameter("publish_odom", true);
    if (!std::isfinite(speed_) || speed_ <= 0 || !std::isfinite(time_scale_) || time_scale_ <= 0) {
      throw std::invalid_argument("speed and time_scale must be finite and positive");
    }
  }

  static bool valid_pose(const geometry_msgs::msg::Pose &pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
  }

  void create_ros_interfaces() {
    // /clock — общий источник времени, остальные интерфейсы поддерживают namespace.
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("state/odom", rclcpp::QoS(5).reliable());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("localization/pose", 5);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    plan_server_ = rclcpp_action::create_server<Plan>(
      this, "planning/compute_path",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Plan::Goal> goal) {
        if (plan_ || goal->goal.header.frame_id != "map" || !valid_pose(goal->goal.pose)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<PlanHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<PlanHandle> handle) {
        plan_ = handle;
        planning_started_ = Steady::now();
      });
    follow_server_ = rclcpp_action::create_server<Follow>(
      this, "control/follow_path",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
        if (follow_ || goal->path.header.frame_id != "map" || goal->path.poses.empty()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        for (const auto &pose : goal->path.poses) {
          if (!valid_pose(pose.pose)) return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<FollowHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<FollowHandle> handle) {
        follow_ = handle;
        path_index_ = 0;
        drifting_ = false;
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "debug/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        reset_episode();
        response->success = true;
        response->message = "debug simulation reset";
      });
  }

  builtin_interfaces::msg::Time stamp() const {
    return static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(static_cast<int64_t>(sim_time_ * 1e9)));
  }

  void on_tick() {
    const double dt = 0.02 * time_scale_;
    const bool frozen = get_parameter("freeze_clock").as_bool();
    if (!frozen) sim_time_ += dt;
    rosgraph_msgs::msg::Clock clock;
    clock.clock = stamp();
    clock_pub_->publish(clock);
    update_planning();
    if (!frozen) update_motion(dt);
    publish_state();
  }

  void update_planning() {
    if (!plan_) return;
    auto result = std::make_shared<Plan::Result>();
    if (plan_->is_canceling()) {
      plan_->canceled(result);
      plan_.reset();
      return;
    }
    const double elapsed = std::chrono::duration<double>(Steady::now() - planning_started_).count();
    if (elapsed < get_parameter("planning_delay").as_double()) return;
    if (get_parameter("blocked_route").as_bool()) {
      result->error_code = Plan::Result::NO_VALID_PATH;
      result->error_msg = "debug blocked route";
      plan_->abort(result);
      plan_.reset();
      return;
    }
    const auto goal = plan_->get_goal();
    const auto start = goal->use_start ? goal->start.pose : pose_;
    const double dx = goal->goal.pose.position.x - start.position.x;
    const double dy = goal->goal.pose.position.y - start.position.y;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / 0.2)));
    result->path.header.frame_id = "map";
    result->path.header.stamp = stamp();
    for (int i = 0; i <= steps; ++i) {
      auto waypoint = goal->goal;
      const double fraction = static_cast<double>(i) / steps;
      waypoint.pose.position.x = start.position.x + fraction * dx;
      waypoint.pose.position.y = start.position.y + fraction * dy;
      waypoint.header = result->path.header;
      result->path.poses.push_back(waypoint);
    }
    result->planning_time = static_cast<builtin_interfaces::msg::Duration>(rclcpp::Duration::from_seconds(elapsed));
    plan_->succeed(result);
    plan_.reset();
  }

  void update_motion(double dt) {
    velocity_ = 0;
    if (!follow_) {
      if (drifting_) {
        velocity_ = speed_;
        pose_.position.x += velocity_ * dt;
      }
      return;
    }
    auto result = std::make_shared<Follow::Result>();
    if (follow_->is_canceling()) {
      drifting_ = get_parameter("failure_to_stop").as_bool();
      velocity_ = drifting_ ? speed_ : 0.0;
      follow_->canceled(result);
      follow_.reset();
      return;
    }
    if (get_parameter("controller_failure").as_bool()) {
      result->error_code = Follow::Result::NO_VALID_CONTROL;
      result->error_msg = "debug injected controller failure";
      follow_->abort(result);
      follow_.reset();
      return;
    }
    const auto &path = follow_->get_goal()->path.poses;
    double remaining_step = speed_ * dt;
    while (path_index_ < path.size()) {
      const auto &target = path[path_index_].pose;
      const double dx = target.position.x - pose_.position.x;
      const double dy = target.position.y - pose_.position.y;
      const double distance = std::hypot(dx, dy);
      if (distance > remaining_step) {
        pose_.position.x += remaining_step * dx / distance;
        pose_.position.y += remaining_step * dy / distance;
        const double yaw = std::atan2(dy, dx);
        pose_.orientation.z = std::sin(yaw / 2);
        pose_.orientation.w = std::cos(yaw / 2);
        velocity_ = speed_;
        break;
      }
      pose_ = target;
      remaining_step -= distance;
      ++path_index_;
    }
    if (path_index_ == path.size()) {
      follow_->succeed(result);
      follow_.reset();
      return;
    }
    auto feedback = std::make_shared<Follow::Feedback>();
    feedback->speed = static_cast<float>(velocity_);
    double distance = std::hypot(path[path_index_].pose.position.x - pose_.position.x,
                                 path[path_index_].pose.position.y - pose_.position.y);
    for (std::size_t i = path_index_ + 1; i < path.size(); ++i) {
      distance += std::hypot(path[i].pose.position.x - path[i - 1].pose.position.x,
                            path[i].pose.position.y - path[i - 1].pose.position.y);
    }
    feedback->distance_to_goal = static_cast<float>(distance);
    follow_->publish_feedback(feedback);
  }

  void publish_state() {
    if (get_parameter("publish_odom").as_bool()) {
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "odom";
      odom.header.stamp = stamp();
      odom.child_frame_id = "base_link";
      odom.pose.pose = pose_;
      odom.twist.twist.linear.x = velocity_;
      odom_pub_->publish(odom);
      geometry_msgs::msg::PoseWithCovarianceStamped localized;
      localized.header = odom.header;
      localized.header.frame_id = "map";
      localized.pose = odom.pose;
      pose_pub_->publish(localized);
    }
    if (++ticks_ % 50 != 0) return;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "capstone_debug_navigation";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "TEST ONLY: kinematic actions, no safety or physical simulation";
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  void reset_episode() {
    if (plan_) {
      plan_->abort(std::make_shared<Plan::Result>());
      plan_.reset();
    }
    if (follow_) {
      follow_->abort(std::make_shared<Follow::Result>());
      follow_.reset();
    }
    pose_ = geometry_msgs::msg::Pose();
    pose_.orientation.w = 1;
    sim_time_ = 0;
    velocity_ = 0;
    drifting_ = false;
  }

  double speed_{1}, time_scale_{5}, sim_time_{0}, velocity_{0};
  bool drifting_{false};
  std::size_t path_index_{0}, ticks_{0};
  geometry_msgs::msg::Pose pose_ = [] { geometry_msgs::msg::Pose p; p.orientation.w = 1; return p; }();
  Steady::time_point planning_started_;
  std::shared_ptr<PlanHandle> plan_;
  std::shared_ptr<FollowHandle> follow_;
  rclcpp_action::Server<Plan>::SharedPtr plan_server_;
  rclcpp_action::Server<Follow>::SharedPtr follow_server_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace capstone_debug_navigation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<capstone_debug_navigation::DebugNavigation>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("debug_navigation"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
