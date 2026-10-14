#pragma once

#include "capstone_control/controller.hpp"

namespace capstone_control {

struct PurePursuitParams {
  double lookahead{0.6};               // m, distance along the path to the target point
  double max_speed{0.5};               // m/s
  double min_speed{0.05};              // m/s, floor while driving so the robot does not stall
  double max_yaw_rate{1.0};            // rad/s
  double curvature_slowdown{1.0};      // v = max_speed / (1 + k * |curvature|)
  double approach_distance{0.8};       // m of remaining path over which speed ramps down
  double rotate_in_place_angle{0.8};   // rad; larger heading error to the target -> turn on the spot
  double final_position_tolerance{0.15};  // m; closer to the goal -> stop and align yaw
  double yaw_gain{1.5};                // w = k * yaw error while turning on the spot
  double min_rotate_speed{0.15};       // rad/s, floor while turning on the spot
  double final_yaw_tolerance{0.1};     // rad; aligned closer than this -> zero command
};

/// Temporary classical tracker used until MPC is ready: steers along a circular arc
/// through a point `lookahead` metres ahead on the path. Ignores obstacles — the safety
/// monitor stops the robot, avoidance is the job of MPC/PPO.
///
/// Phases: turn on the spot if the target is far to the side or behind; drive along the
/// arc otherwise; once within `final_position_tolerance` of the goal, stop and turn to the
/// goal yaw. The last phase is latched until reset().
class PurePursuitController : public Controller {
 public:
  explicit PurePursuitController(PurePursuitParams params = {});

  void reset() override;
  std::optional<Command> compute(const ControlInput &input) override;

  const PurePursuitParams &params() const { return params_; }
  bool aligning() const { return aligning_; }

 private:
  Command rotate_towards(double yaw_error) const;

  PurePursuitParams params_;
  bool aligning_{false};
};

}  // namespace capstone_control
