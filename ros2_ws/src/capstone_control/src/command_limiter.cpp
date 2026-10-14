#include "capstone_control/command_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capstone_control {

namespace {

double rate_limit(double desired, double previous, double up, double down, double dt) {
  // Slowing down means moving towards zero without crossing it.
  const bool slowing = std::abs(desired) < std::abs(previous) && desired * previous >= 0.0;
  const double step = (slowing ? down : up) * dt;
  return previous + std::clamp(desired - previous, -step, step);
}

}  // namespace

CommandLimiter::CommandLimiter(CommandLimits limits) : limits_(limits) {
  if (!(limits_.max_speed > 0.0) || limits_.max_reverse_speed < 0.0 || !(limits_.max_yaw_rate > 0.0) ||
      !(limits_.max_accel > 0.0) || !(limits_.max_decel > 0.0) || !(limits_.max_yaw_accel > 0.0)) {
    throw std::invalid_argument("invalid command limits");
  }
}

Command CommandLimiter::limit(const Command &desired, const Command &previous, double dt) const {
  if (!is_finite(desired)) {
    return Command{0.0, 0.0};
  }
  const Command prev = is_finite(previous) ? previous : Command{0.0, 0.0};
  const double step_dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.0;

  Command out;
  out.v = std::clamp(desired.v, -limits_.max_reverse_speed, limits_.max_speed);
  out.w = std::clamp(desired.w, -limits_.max_yaw_rate, limits_.max_yaw_rate);
  out.v = rate_limit(out.v, prev.v, limits_.max_accel, limits_.max_decel, step_dt);
  out.w = rate_limit(out.w, prev.w, limits_.max_yaw_accel, limits_.max_yaw_accel, step_dt);
  // A previous command above a (changed) limit must not leak through the rate limit.
  out.v = std::clamp(out.v, -limits_.max_reverse_speed, limits_.max_speed);
  out.w = std::clamp(out.w, -limits_.max_yaw_rate, limits_.max_yaw_rate);
  return out;
}

}  // namespace capstone_control
