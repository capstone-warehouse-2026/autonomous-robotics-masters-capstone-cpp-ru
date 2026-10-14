#pragma once

#include "capstone_control/geometry.hpp"

namespace capstone_control {

/// Success condition of FollowPath from docs/INTERFACES.md:
/// position within 0.25 m, yaw within 0.2 rad, robot (almost) stopped.
struct GoalTolerance {
  double xy{0.25};             // m
  double yaw{0.2};             // rad
  double stopped_speed{0.05};  // m/s, |v| below this counts as stopped
  double stopped_yaw_rate{0.1};  // rad/s, |w| below this counts as stopped
};

class GoalChecker {
 public:
  explicit GoalChecker(GoalTolerance tolerance = {});
  bool reached(const Pose2D &robot, const Pose2D &goal, const Command &measured_velocity) const;
  const GoalTolerance &tolerance() const { return tolerance_; }

 private:
  GoalTolerance tolerance_;
};

struct ProgressParams {
  double required_movement{0.1};  // m the robot must move ...
  double required_rotation{0.5};  // ... or rad it must turn ...
  double time_allowance{10.0};    // ... within this many seconds
};

/// Detects a stuck robot: no movement of `required_movement` metres and no turn of
/// `required_rotation` radians for `time_allowance` seconds. Time is simulation time.
class ProgressChecker {
 public:
  explicit ProgressChecker(ProgressParams params = {});

  /// Starts a new observation window (new goal, or after a pause).
  void reset(const Pose2D &robot, double now);

  /// Returns false when the robot is considered stuck.
  bool check(const Pose2D &robot, double now);

  const ProgressParams &params() const { return params_; }

 private:
  ProgressParams params_;
  Pose2D anchor_;
  double anchor_time_{0.0};
  bool started_{false};
};

}  // namespace capstone_control
