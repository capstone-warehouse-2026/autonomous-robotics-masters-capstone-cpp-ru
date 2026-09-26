#include "capstone_sim/diff_drive.hpp"

#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

namespace capstone_sim {

namespace {

const std::string &required(const std::unordered_map<std::string, std::string> &parameters,
                            const std::string &name) {
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    throw std::runtime_error("DiffDrive: URDF parameter <" + name + "> is missing");
  }
  return it->second;
}

double positive(const std::unordered_map<std::string, std::string> &parameters, const std::string &name) {
  const double value = std::stod(required(parameters, name));
  if (!(value > 0.0)) {
    throw std::runtime_error("DiffDrive: URDF parameter <" + name + "> must be positive");
  }
  return value;
}

WbDeviceTag velocity_motor(const std::string &name) {
  const WbDeviceTag motor = wb_robot_get_device(name.c_str());
  if (motor == 0) {
    throw std::runtime_error("DiffDrive: motor '" + name + "' not found in the Webots world");
  }
  wb_motor_set_position(motor, INFINITY);  // velocity control
  wb_motor_set_velocity(motor, 0.0);
  return motor;
}

}  // namespace

void DiffDrive::init(webots_ros2_driver::WebotsNode *node,
                     std::unordered_map<std::string, std::string> &parameters) {
  node_ = node;
  left_motor_ = velocity_motor(required(parameters, "leftMotor"));
  right_motor_ = velocity_motor(required(parameters, "rightMotor"));
  wheel_radius_ = positive(parameters, "wheelRadius");
  wheel_separation_ = positive(parameters, "wheelSeparation");
  timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(positive(parameters, "cmdVelTimeout")));
  max_wheel_speed_ = std::min(wb_motor_get_max_velocity(left_motor_), wb_motor_get_max_velocity(right_motor_));

  subscription_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::TwistStamped &message) { on_cmd_vel(message); });
  RCLCPP_INFO(node_->get_logger(), "diff drive ready: /cmd_vel, watchdog %.0f ms, max %.2f m/s",
              std::chrono::duration<double, std::milli>(timeout_).count(), max_wheel_speed_ * wheel_radius_);
}

void DiffDrive::on_cmd_vel(const geometry_msgs::msg::TwistStamped &message) {
  const auto &twist = message.twist;
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
    RCLCPP_WARN(node_->get_logger(), "rejected non-finite /cmd_vel, stopping");
    command_time_.reset();
    return;
  }
  command_ = twist;
  command_time_ = std::chrono::steady_clock::now();
}

void DiffDrive::step() {
  // Callbacks and step() run in the same executor thread of WebotsNode, so no locking is needed.
  const bool fresh = command_time_ && std::chrono::steady_clock::now() - *command_time_ <= timeout_;
  if (!fresh) {
    if (!stopped_) {
      RCLCPP_WARN(node_->get_logger(), "no fresh /cmd_vel, wheels stopped");
    }
    set_wheel_speeds({});
    stopped_ = true;
    return;
  }
  stopped_ = false;
  set_wheel_speeds(limit_wheel_speeds(
      diff_drive_wheel_speeds(command_.linear.x, command_.angular.z, wheel_radius_, wheel_separation_),
      max_wheel_speed_));
}

void DiffDrive::set_wheel_speeds(const WheelSpeeds &speeds) {
  wb_motor_set_velocity(left_motor_, speeds.left);
  wb_motor_set_velocity(right_motor_, speeds.right);
}

}  // namespace capstone_sim

PLUGINLIB_EXPORT_CLASS(capstone_sim::DiffDrive, webots_ros2_driver::PluginInterface)
