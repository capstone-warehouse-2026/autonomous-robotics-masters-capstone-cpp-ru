#pragma once

#include <optional>

#include "capstone_control/geometry.hpp"
#include "capstone_control/path_tracker.hpp"

namespace capstone_control {

/// Everything a local controller may use in one control cycle.
/// Both the classical (MPC) and the learned (PPO) controllers receive the same input,
/// so neither has access to information the other lacks (projects/04_control.md).
struct ControlInput {
  Pose2D pose;               // robot pose in the path frame (`map`)
  Command velocity;          // measured velocity from odometry
  const PathTracker *path{nullptr};  // already updated with `pose`
  double dt{0.05};           // control period, s
  // TODO(G4): local obstacle grid / scan ranges for MPC and PPO.
};

/// Common interface of all local controllers. Selected by the `method` parameter.
class Controller {
 public:
  virtual ~Controller() = default;

  /// Called when a new goal is accepted; drops any state of the previous goal.
  virtual void reset() = 0;

  /// Desired command before rate limiting, or std::nullopt when the controller cannot
  /// produce a valid command this cycle (infeasible MPC, non-finite network output, ...).
  /// The caller then publishes zero and counts the failure (ADR 0004).
  virtual std::optional<Command> compute(const ControlInput &input) = 0;
};

}  // namespace capstone_control
