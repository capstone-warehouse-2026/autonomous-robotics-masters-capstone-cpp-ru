#include "capstone_control/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capstone_control {

PurePursuitController::PurePursuitController(PurePursuitParams params) : params_(params) {
  if (!(params_.lookahead > 0.0) || !(params_.max_speed > 0.0) || !(params_.max_yaw_rate > 0.0) ||
      params_.min_speed < 0.0 || params_.min_speed > params_.max_speed || !(params_.approach_distance > 0.0)) {
    throw std::invalid_argument("invalid pure pursuit parameters");
  }
}

void PurePursuitController::reset() { aligning_ = false; }

Command PurePursuitController::rotate_towards(double yaw_error) const {
  if (std::abs(yaw_error) < params_.final_yaw_tolerance) {
    return Command{0.0, 0.0};
  }
  double w = params_.yaw_gain * yaw_error;
  w = std::copysign(std::clamp(std::abs(w), params_.min_rotate_speed, params_.max_yaw_rate), yaw_error);
  return Command{0.0, w};
}

std::optional<Command> PurePursuitController::compute(const ControlInput &input) {
  if (input.path == nullptr || input.path->empty() || !is_finite(input.pose)) {
    return std::nullopt;
  }
  const PathTracker &path = *input.path;
  const Pose2D &robot = input.pose;
  const Pose2D &goal = path.goal();

  // Final phase: close to the goal position, only fix the heading.
  if (!aligning_ && distance(robot, goal) < params_.final_position_tolerance) {
    aligning_ = true;
  }
  if (aligning_) {
    return rotate_towards(normalize_angle(goal.yaw - robot.yaw));
  }

  const Pose2D target = path.point_ahead(params_.lookahead);
  const Point2D t = to_robot_frame(robot, target.x, target.y);
  const double dist2 = t.x * t.x + t.y * t.y;
  if (dist2 < 1e-6) {
    return Command{0.0, 0.0};
  }

  // Target behind or far to the side: turning on the spot is better than a wide arc.
  const double heading_error = std::atan2(t.y, t.x);
  if (std::abs(heading_error) > params_.rotate_in_place_angle) {
    return rotate_towards(heading_error);
  }

  const double curvature = 2.0 * t.y / dist2;
  double v = params_.max_speed / (1.0 + params_.curvature_slowdown * std::abs(curvature));
  const double ramp = params_.max_speed * path.remaining_length() / params_.approach_distance;
  v = std::max(std::min(v, ramp), params_.min_speed);
  double w = v * curvature;
  if (std::abs(w) > params_.max_yaw_rate) {
    // Keep the arc, lower the speed.
    w = std::copysign(params_.max_yaw_rate, w);
    v = params_.max_yaw_rate / std::abs(curvature);
  }

  const Command cmd{v, w};
  if (!is_finite(cmd)) {
    return std::nullopt;
  }
  return cmd;
}

}  // namespace capstone_control
