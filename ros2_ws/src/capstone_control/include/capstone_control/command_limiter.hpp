#pragma once

#include "capstone_control/geometry.hpp"

namespace capstone_control {

struct CommandLimits {
  double max_speed{0.5};          // m/s forward
  double max_reverse_speed{0.0};  // m/s backwards (0 = never reverse)
  double max_yaw_rate{1.0};       // rad/s
  double max_accel{0.5};          // m/s^2 when speeding up
  double max_decel{1.0};          // m/s^2 when slowing down
  double max_yaw_accel{2.0};      // rad/s^2
};

/// Clamps a desired command to speed limits and to the change allowed in one cycle.
/// Applied to every controller (pure pursuit, MPC, PPO) so they are compared under
/// the same actuation limits. Not a safety function: the monitor checks independently.
class CommandLimiter {
 public:
  explicit CommandLimiter(CommandLimits limits = {});

  /// `previous` is the last command sent, `dt` the time since it (s, > 0).
  /// A non-finite desired command yields zero.
  Command limit(const Command &desired, const Command &previous, double dt) const;

  const CommandLimits &limits() const { return limits_; }

 private:
  CommandLimits limits_;
};

}  // namespace capstone_control
