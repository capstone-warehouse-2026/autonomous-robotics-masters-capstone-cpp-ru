#pragma once

#include <atomic>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/grid.hpp"
#include "capstone_planning/grid_search.hpp"

namespace capstone_planning {

/// nav_msgs/OccupancyGrid -> Grid (origin yaw from the quaternion). Throws std::invalid_argument
/// for an empty grid, a data size mismatch or a non-finite / non-positive resolution.
Grid grid_from_message(const nav_msgs::msg::OccupancyGrid &message);

/// Yaw of a quaternion (rotation about z of a planar pose).
double yaw_from_quaternion(double x, double y, double z, double w);

/// G3 global planner (projects/03_planning.md, docs/INTERFACES.md).
///
/// Inputs: /map (static map, transient local), /perception/obstacles (un-inflated local grid of G2,
/// usually in odom), /localization/pose (start when the goal has use_start = false), TF map <- odom.
/// Output: action server /planning/compute_path (nav2_msgs/action/ComputePathToPose); on success the
/// result holds a nav_msgs/Path in `map`. Every failure is an aborted goal with a nav2 error code and
/// a message, never an empty "successful" path. /diagnostics at 1 Hz with the planning metrics.
///
/// Each goal fuses the latest static map and local grid, inflates them with the shared robot
/// footprint (CostMap) and runs A* in a worker thread with the 200 ms budget; cancel and a newer goal
/// (preemption) stop the search. The last successful path is re-checked against every fresh local
/// grid and reported on /diagnostics when it becomes blocked, so that G5 can ask for a new path.
///
/// `method: classical|learned` selects the cost. The learned cost (method B) is not implemented yet:
/// `learned` runs the classical cost as its fallback and reports that on /diagnostics.
class PlanningNode : public rclcpp::Node {
 public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePath>;

  explicit PlanningNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PlanningNode() override;

  const std::string &method() const { return method_; }
  bool has_map() const;

  /// Plans one request synchronously (the action worker uses it; tests may call it directly).
  /// `cancel` is polled during the search.
  ComputePath::Result plan(const ComputePath::Goal &goal, const std::function<bool()> &cancel = {});

 private:
  struct LocalGrid {
    Grid grid;
    std::string frame;
    rclcpp::Time stamp;
  };

  // Action server.
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ComputePath::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void on_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void worker_loop();

  // Inputs.
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void on_obstacles(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);

  // Planning helpers.
  std::optional<Pose2D> to_map_frame(const geometry_msgs::msg::PoseStamped &pose, std::string &error);
  std::shared_ptr<const CostMap> build_cost_map(std::string &error, uint16_t &error_code);
  void record(const ComputePath::Result &result, double latency_ms, std::size_t expanded);
  void check_last_path();
  void publish_diagnostics();

  // Parameters.
  std::string method_;
  std::string map_frame_;
  CostParams cost_params_;
  double planning_timeout_ = 0.2;   // s, steady clock
  double max_obstacles_age_ = 0.3;  // s, sim time
  double max_pose_age_ = 0.3;       // s, sim time
  double tf_timeout_ = 0.05;        // s
  double start_escape_radius_ = 0.5;
  bool require_obstacles_ = true;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacles_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp_action::Server<ComputePath>::SharedPtr action_server_;

  // Latest inputs, guarded by inputs_mutex_.
  mutable std::mutex inputs_mutex_;
  std::shared_ptr<const Grid> static_map_;
  std::shared_ptr<const LocalGrid> local_grid_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose_;
  std::string last_input_error_;

  // Goal queue: one goal runs at a time; a newer goal preempts the running and the pending one.
  std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  std::shared_ptr<GoalHandle> pending_goal_;
  std::atomic<bool> preempt_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;

  // Metrics and the last path, guarded by stats_mutex_.
  std::mutex stats_mutex_;
  std::deque<double> latencies_ms_;  // last kLatencyWindow requests, success or not
  std::size_t requests_ = 0;
  std::size_t successes_ = 0;
  std::size_t deadline_misses_ = 0;
  std::size_t learned_fallbacks_ = 0;
  std::size_t path_blocked_events_ = 0;
  std::map<uint16_t, std::size_t> failures_by_code_;
  std::size_t last_expanded_ = 0;
  double last_length_ = 0.0;
  double last_min_clearance_ = 0.0;
  std::string last_error_;
  std::vector<Pose2D> last_path_;
  bool last_path_blocked_ = false;
  std::chrono::steady_clock::time_point last_path_check_{};
};

}  // namespace capstone_planning
