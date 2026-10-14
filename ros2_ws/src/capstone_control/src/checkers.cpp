#include "capstone_control/checkers.hpp"

#include <cmath>
#include <stdexcept>

namespace capstone_control {

GoalChecker::GoalChecker(GoalTolerance tolerance) : tolerance_(tolerance) {
  if (!(tolerance_.xy > 0.0) || !(tolerance_.yaw > 0.0) || !(tolerance_.stopped_speed > 0.0) ||
      !(tolerance_.stopped_yaw_rate > 0.0)) {
    throw std::invalid_argument("invalid goal tolerance");
  }
}

bool GoalChecker::reached(const Pose2D &robot, const Pose2D &goal, const Command &measured_velocity) const {
  if (!is_finite(robot) || !is_finite(goal) || !is_finite(measured_velocity)) {
    return false;
  }
  return distance(robot, goal) <= tolerance_.xy &&
         std::abs(normalize_angle(goal.yaw - robot.yaw)) <= tolerance_.yaw &&
         std::abs(measured_velocity.v) < tolerance_.stopped_speed &&
         std::abs(measured_velocity.w) < tolerance_.stopped_yaw_rate;
}

ProgressChecker::ProgressChecker(ProgressParams params) : params_(params) {
  if (!(params_.required_movement > 0.0) || !(params_.required_rotation > 0.0) ||
      !(params_.time_allowance > 0.0)) {
    throw std::invalid_argument("invalid progress checker parameters");
  }
}

void ProgressChecker::reset(const Pose2D &robot, double now) {
  anchor_ = robot;
  anchor_time_ = now;
  started_ = true;
}

bool ProgressChecker::check(const Pose2D &robot, double now) {
  if (!started_ || now < anchor_time_) {
    // First call, or simulation time went backwards (reset): start a new window.
    reset(robot, now);
    return true;
  }
  if (distance(anchor_, robot) >= params_.required_movement ||
      std::abs(normalize_angle(robot.yaw - anchor_.yaw)) >= params_.required_rotation) {
    reset(robot, now);
    return true;
  }
  return now - anchor_time_ <= params_.time_allowance;
}

}  // namespace capstone_control
