#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <webots/types.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include "capstone_sim/diff_drive_kinematics.hpp"

namespace capstone_sim {

// Simulator adapter of a differential-drive base (G1): the only consumer of /cmd_vel in simulation.
// Motor names and wheel geometry come from the robot URDF (resource/*.urdf). Stops the wheels when
// commands are stale or not finite, independently of the safety monitor (docs/INTERFACES.md).
class DiffDrive : public webots_ros2_driver::PluginInterface {
 public:
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
  void step() override;

 private:
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped &message);
  void set_wheel_speeds(const WheelSpeeds &speeds);

  webots_ros2_driver::WebotsNode *node_ = nullptr;
  WbDeviceTag left_motor_ = 0;
  WbDeviceTag right_motor_ = 0;
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;
  double max_wheel_speed_ = 0.0;
  std::chrono::steady_clock::duration timeout_ = std::chrono::milliseconds(300);
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  geometry_msgs::msg::Twist command_;
  std::optional<std::chrono::steady_clock::time_point> command_time_;
  bool stopped_ = true;
};

}  // namespace capstone_sim
