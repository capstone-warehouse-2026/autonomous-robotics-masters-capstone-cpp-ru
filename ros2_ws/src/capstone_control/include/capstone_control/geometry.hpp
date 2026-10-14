#pragma once

// Plain 2D types shared by the control algorithms. No ROS dependency, so the
// algorithms can be unit-tested and reused by MPC/PPO without a running graph.

#include <cmath>
#include <vector>

namespace capstone_control {

/// Robot or path pose in a planar frame (normally `map`). Units: m, rad.
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Velocity command for a differential-drive robot in `base_link`.
/// v — forward speed, m/s; w — yaw rate, rad/s (positive = counter-clockwise).
struct Command {
  double v{0.0};
  double w{0.0};
};

using Path = std::vector<Pose2D>;

constexpr double kPi = 3.14159265358979323846;

/// Wraps an angle to [-pi, pi).
inline double normalize_angle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

inline double distance(const Pose2D &a, const Pose2D &b) { return std::hypot(b.x - a.x, b.y - a.y); }

inline bool is_finite(const Pose2D &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}

inline bool is_finite(const Command &c) { return std::isfinite(c.v) && std::isfinite(c.w); }

/// Expresses a world-frame point in the robot frame (x forward, y left).
inline void to_robot_frame(const Pose2D &robot, double wx, double wy, double &rx, double &ry) {
  const double dx = wx - robot.x;
  const double dy = wy - robot.y;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  rx = c * dx + s * dy;
  ry = -s * dx + c * dy;
}

}  // namespace capstone_control
