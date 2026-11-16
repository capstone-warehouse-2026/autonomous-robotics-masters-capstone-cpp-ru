#pragma once

// Network input and output of method B (PPO): a literal copy of ml/control/spec.py. The network was
// trained on observations built by spec.build_observation(); if this code differs, the network in the robot
// is not the network that was evaluated. test_ppo_observation.cpp checks the same numbers as test_spec.py.

#include <array>
#include <vector>

#include "capstone_control/controller.hpp"
#include "capstone_control/geometry.hpp"

namespace capstone_control::ppo {

// Robot limits, identical to spec.py, MpcParams and CommandLimits (docs/adr/0004).
constexpr double kVMax = 0.5;  // m/s
constexpr double kWMax = 1.0;  // rad/s

// Observation layout, see the docstring of spec.py.
constexpr int kPathPoints = 10;
constexpr double kPathSpacing = 0.2;   // m
constexpr double kPathScale = 2.0;     // m
constexpr double kRemainingMax = 5.0;  // m
constexpr int kSectors = 36;
constexpr double kRangeMax = 3.0;      // m
constexpr int kObsSize = 2 * kPathPoints + 1 + 2 + kSectors + 2;
constexpr int kActionSize = 2;

// The primitive path inputs of spec.path_features().
struct PathFeatures {
  std::array<Point2D, kPathPoints> points;  // path->point_ahead((i + 1) * kPathSpacing) in base_link
  double remaining = 0.0;                   // path->remaining_length()
  double goal_yaw = 0.0;                    // normalize_angle(goal().yaw - pose.yaw)
};

// spec.path_features(): from the PathTracker of the input (non-null, already updated with input.pose).
PathFeatures path_features(const ControlInput &input);

// spec.build_observation(features, obstacles, v, w); kObsSize floats.
std::vector<float> build_observation(const PathFeatures &path, const std::vector<Point2D> &obstacles,
                                     const Command &velocity);

// Both steps for a ControlInput.
std::vector<float> build_observation(const ControlInput &input);

// spec.action_to_target(): clip to [-1, 1], v = (a0 + 1) / 2 * kVMax, w = a1 * kWMax. The CommandLimiter of
// the node applies the acceleration limits afterwards.
Command action_to_target(float a0, float a1);

}  // namespace capstone_control::ppo
