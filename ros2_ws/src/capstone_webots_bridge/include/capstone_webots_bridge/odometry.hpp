#pragma once

#include <cmath>
#include <stdexcept>

namespace capstone_webots_bridge {

// Положение получается только из измеренных углов колёс, без данных Supervisor.
class WheelOdometry {
 public:
  WheelOdometry(double radius, double separation) : radius_(radius), separation_(separation) {
    if (!std::isfinite(radius) || !std::isfinite(separation) || radius <= 0 || separation <= 0) {
      throw std::invalid_argument("invalid differential drive geometry");
    }
  }
  void reset() { initialized_ = false; x = y = yaw = velocity = yaw_rate = distance = 0.0; }
  bool update(double left, double right, double dt) {
    if (!std::isfinite(left) || !std::isfinite(right) || !std::isfinite(dt) || dt <= 0) return false;
    if (!initialized_) {
      last_left_ = left;
      last_right_ = right;
      initialized_ = true;
      return true;
    }
    const double dl = (left - last_left_) * radius_;
    const double dr = (right - last_right_) * radius_;
    last_left_ = left;
    last_right_ = right;
    const double ds = (dl + dr) * 0.5;
    const double angle = (dr - dl) / separation_;
    const double scale = std::abs(angle) < 1e-9 ? 1.0 : 2 * std::sin(angle * 0.5) / angle;
    x += ds * scale * std::cos(yaw + angle * 0.5);
    y += ds * scale * std::sin(yaw + angle * 0.5);
    yaw = std::atan2(std::sin(yaw + angle), std::cos(yaw + angle));
    velocity = ds / dt;
    yaw_rate = angle / dt;
    distance += std::abs(ds);
    return true;
  }
  double x{0}, y{0}, yaw{0}, velocity{0}, yaw_rate{0}, distance{0};
 private:
  double radius_, separation_, last_left_{0}, last_right_{0};
  bool initialized_{false};
};

inline bool command_is_fresh(double stamp, double sim_time, double wall_age,
                             double velocity, double yaw_rate) {
  return std::isfinite(stamp) && std::isfinite(sim_time) && std::isfinite(wall_age) &&
         std::isfinite(velocity) && std::isfinite(yaw_rate) && stamp <= sim_time + 0.02 &&
         sim_time - stamp <= 0.3 && wall_age >= 0 && wall_age <= 0.3 &&
         std::abs(velocity) <= 0.62 && std::abs(yaw_rate) <= 1.22;
}
}  // namespace capstone_webots_bridge
