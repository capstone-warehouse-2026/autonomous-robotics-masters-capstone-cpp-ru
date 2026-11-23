#pragma once

#include <vector>

#include "capstone_integration/kinematics.hpp"

namespace capstone_integration {

// Parameters of the placeholder path follower. They are deliberately simple: the goal is to make
// the interfaces live end-to-end, not to compete with G4's MPC/PPO.
struct FollowerParams {
  double lookahead_m{0.8};
  double max_v{0.5};
  double max_w{1.0};
  double heading_gain{1.5};
  double turn_in_place_angle_rad{0.6};
  double arrive_radius_m{0.15};
};

struct ControlCommand {
  double v{0.0};
  double w{0.0};
};

// Samples a straight segment from `start` to `goal` every `step_m`; the last pose carries
// `goal.yaw` so the robot can align with the station.
std::vector<Pose2D> straight_path(const Pose2D &start, const Pose2D &goal, double step_m);

double path_length(const std::vector<Pose2D> &path);

// Length of the remaining path from the projection of `pose` onto the path to its end.
double remaining_distance(const std::vector<Pose2D> &path, const Pose2D &pose);

// One control step: turn in place when the heading error is large, otherwise drive while steering
// towards a look-ahead point. Stops inside `arrive_radius_m` of the path end.
ControlCommand follow_path(const std::vector<Pose2D> &path, const Pose2D &pose,
                           const FollowerParams &params);

}  // namespace capstone_integration
