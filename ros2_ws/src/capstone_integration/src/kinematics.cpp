#include "capstone_integration/kinematics.hpp"

#include <algorithm>
#include <stdexcept>

namespace capstone_integration {

double normalize_angle(double angle) {
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("normalize_angle: angle is not finite");
  }
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

double angle_diff(double target, double current) {
  return normalize_angle(target - current);
}

Pose2D integrate_twist(const Pose2D &pose, double v, double w, double dt) {
  if (!std::isfinite(v) || !std::isfinite(w) || !std::isfinite(dt) || dt <= 0.0) {
    return pose;
  }
  // Exact arc integration; degenerate case w == 0 falls back to a straight line.
  if (std::abs(w) < 1e-9) {
    return Pose2D{pose.x + v * dt * std::cos(pose.yaw),
                  pose.y + v * dt * std::sin(pose.yaw),
                  pose.yaw};
  }
  const double yaw = pose.yaw + w * dt;
  const double radius = v / w;
  return Pose2D{pose.x + radius * (std::sin(yaw) - std::sin(pose.yaw)),
                pose.y - radius * (std::cos(yaw) - std::cos(pose.yaw)),
                yaw};
}

double distance(const Pose2D &a, const Pose2D &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

bool goal_reached(const Pose2D &pose, const Pose2D &goal, double position_tolerance,
                  double yaw_tolerance) {
  if (position_tolerance <= 0.0 || yaw_tolerance <= 0.0) {
    return false;
  }
  return distance(pose, goal) <= position_tolerance &&
         std::abs(angle_diff(goal.yaw, pose.yaw)) <= yaw_tolerance;
}

double clamp(double value, double low, double high) {
  if (low > high) {
    throw std::invalid_argument("clamp: low > high");
  }
  return std::min(std::max(value, low), high);
}

}  // namespace capstone_integration
