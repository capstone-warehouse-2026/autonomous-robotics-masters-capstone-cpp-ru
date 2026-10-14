#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "capstone_safety/safety_core.hpp"

namespace capstone_safety {

// Planar pose of a sensor (the LiDAR) in base_link.
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Converts a LaserScan into points in base_link; sensor_in_base is the pose of the scan frame.
// Rays outside [range_min, range_max] and NaN/Inf rays are dropped: they are "no return", not obstacles.
std::vector<Point> scan_to_points(const sensor_msgs::msg::LaserScan &scan, const Pose2D &sensor_in_base);

// Independent G4 safety monitor, the only publisher of /cmd_vel, shared by both control methods.
// A thin ROS wrapper around SafetyCore: subscriptions to /control/cmd_vel_raw, /state/odom and
// /sensors/scan feed the core, a 20 Hz wall timer calls SafetyCore::step() and publishes the result on
// /cmd_vel every tick, zeros included; the reason is reported on /diagnostics. docs/INTERFACES.md.
//
// Lifecycle (docs/INTERFACES.md: configure -> ready -> active -> fault/stopped):
//   configure:  reads the parameters, creates subscriptions, publishers and TF; bad parameters -> FAILURE.
//   activate:   resets the core (no data from before activation is used) and starts the 20 Hz timer.
//   deactivate: publishes one zero command and falls silent; the driver watchdog then stops the robot.
// While not active the node does not publish /cmd_vel at all.
class SafetyMonitorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SafetyMonitorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }
  bool autostart() const { return autostart_; }

 protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

 private:
  Params declare_core_params();  // constructor: declares the parameters, the core validates them
  Params read_core_params();     // configure: reads their current values
  void release_interfaces();
  void publish_stop();

  void on_cmd(const geometry_msgs::msg::TwistStamped &msg);
  void on_odom(const nav_msgs::msg::Odometry &msg);
  void on_scan(const sensor_msgs::msg::LaserScan &msg);
  void on_timer();
  void publish_diagnostics(const Decision &d);

  bool sensor_pose(const std::string &frame, Pose2D &out);
  double steady_now();

  // Declaration order matters: the constructor initializes these members in this order.
  std::string method_;
  std::string base_frame_;
  bool autostart_ = true;
  double rate_hz_ = 20.0;
  SafetyCore core_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string sensor_frame_;            // scan frame of the cached pose
  std::optional<Pose2D> sensor_pose_;   // the LiDAR is rigidly mounted, so its pose is cached

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Statistics for /diagnostics and the intervention rate metric of G5.
  Decision last_decision_;
  std::optional<Reason> last_reason_;  // empty before the first tick, so that tick reports diagnostics
  double last_in_v_ = 0.0;
  double last_in_w_ = 0.0;
  std::uint64_t ticks_ = 0;
  std::map<Reason, std::uint64_t> reason_counts_;
};

}  // namespace capstone_safety
