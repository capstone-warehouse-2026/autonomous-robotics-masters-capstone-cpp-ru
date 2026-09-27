#pragma once

#include <algorithm>
#include <cmath>

namespace capstone_sim {

struct WheelSpeeds {
  double left = 0.0;   // rad/s
  double right = 0.0;  // rad/s
};

// Wheel angular velocities of a differential drive for a body twist in the ROS convention:
// v forward (m/s), w counterclockwise seen from above (rad/s).
inline WheelSpeeds diff_drive_wheel_speeds(double v, double w, double wheel_radius, double wheel_separation) {
  return {(v - w * wheel_separation / 2.0) / wheel_radius, (v + w * wheel_separation / 2.0) / wheel_radius};
}

// Scales both wheels by the same factor so that none exceeds max_speed; keeps the path curvature.
inline WheelSpeeds limit_wheel_speeds(const WheelSpeeds &speeds, double max_speed) {
  const double largest = std::max(std::abs(speeds.left), std::abs(speeds.right));
  if (largest <= max_speed || largest == 0.0) {
    return speeds;
  }
  const double scale = max_speed / largest;
  return {speeds.left * scale, speeds.right * scale};
}

}  // namespace capstone_sim
