#pragma once

#include <cmath>

namespace capstone_integration {

// Pose on the plane, SI units: metres and radians (yaw, counter-clockwise, 0 = +x).
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

constexpr double kPi = 3.14159265358979323846;

// Wraps an angle to (-pi, pi].
double normalize_angle(double angle);

// Shortest signed rotation from `current` to `target`, in (-pi, pi].
double angle_diff(double target, double current);

// Exact unicycle integration of one command held for `dt` seconds.
Pose2D integrate_twist(const Pose2D &pose, double v, double w, double dt);

double distance(const Pose2D &a, const Pose2D &b);

// Position and yaw within tolerance. Used by the placeholder controller and the mission check.
bool goal_reached(const Pose2D &pose, const Pose2D &goal, double position_tolerance,
                  double yaw_tolerance);

double clamp(double value, double low, double high);

}  // namespace capstone_integration
