#include "capstone_safety/safety_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capstone_safety {

std::string to_string(Reason r) {
  // No default: -Wswitch then reports a Reason added to the enum but missing here.
  switch (r) {
    case Reason::OK:              return "OK";
    case Reason::TIME_JUMP:       return "TIME_JUMP";
    case Reason::NOT_READY:       return "NOT_READY";
    case Reason::STALE_SCAN:      return "STALE_SCAN";
    case Reason::STALE_ODOM:      return "STALE_ODOM";
    case Reason::STALE_CMD:       return "STALE_CMD";
    case Reason::NONFINITE:       return "NONFINITE";
    case Reason::LIMIT:           return "LIMIT";
    case Reason::COLLISION_RISK:  return "COLLISION_RISK";
  }
  return "UNKNOWN";
}

SafetyCore::SafetyCore(const Params &params) : params_(params) {
  for (double x : {params.v_max,
                   params.w_max,
                   params.limit_tol,
                   params.a_brake,
                   params.t_latency,
                   params.robot_radius,
                   params.margin,
                   params.path_step,
                   params.v_eps,
                   params.cmd_timeout_sim,
                   params.cmd_timeout_steady,
                   params.sensor_timeout_sim,
                   params.time_jump_tol}) {
    if (!std::isfinite(x)) throw std::invalid_argument("parameter is not finite");
  }

  if (params.a_brake <= 0.0) throw std::invalid_argument("a_brake must be > 0");
  if (params.path_step <= 0.0) throw std::invalid_argument("path_step must be > 0");
  if (params.robot_radius <= 0.0) throw std::invalid_argument("robot_radius must be > 0");
  if (params.v_max <= 0.0) throw std::invalid_argument("v_max must be > 0");
  if (params.w_max <= 0.0) throw std::invalid_argument("w_max must be > 0");
  if (params.t_latency < 0.0) throw std::invalid_argument("t_latency must be >= 0");
  if (params.margin < 0.0) throw std::invalid_argument("margin must be >= 0");
  if (params.cmd_timeout_sim <= 0.0) throw std::invalid_argument("cmd_timeout_sim must be > 0");
  if (params.cmd_timeout_steady <= 0.0) throw std::invalid_argument("cmd_timeout_steady must be > 0");
  if (params.sensor_timeout_sim <= 0.0) throw std::invalid_argument("sensor_timeout_sim must be > 0");

  if (params.limit_tol < 0.0 || params.v_eps < 0.0 || params.time_jump_tol < 0.0)
    throw std::invalid_argument("tolerances must be >= 0");
}

void SafetyCore::reset() {
  cmd_.reset();
  odom_.reset();
  scan_.reset();
  latched_ = false;
}

void SafetyCore::on_cmd(const Cmd &cmd) {
  cmd_ = cmd;
  // Only a command newer than the one that went stale releases the latch; fresh odom or scan
  // say nothing about the controller.
  if (latched_ && cmd.t_sim > latched_stamp_) latched_ = false;
}

void SafetyCore::on_odom(const Odom &odom) {
  odom_ = odom;
}

void SafetyCore::on_scan(const Scan &scan) {
  scan_ = scan;
}

double SafetyCore::stopping_distance(double v, const Params &p) {
  if (v <= 0.0) return 0.0;
  return v * p.t_latency + v * v / (2 * p.a_brake);
}

bool SafetyCore::collision_risk(double v, double w, const std::vector<Point> &points,
                                const Params &p) {
  const double R = p.robot_radius;             // robot disc
  const double C = p.robot_radius + p.margin;  // clearance that must stay free along the path

  // A point inside the robot disc is a contact whatever the command.
  for (const Point &pt : points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
    if (std::hypot(pt.x, pt.y) < R) return true;
  }

  // Standing or turning in place: a disc does not move towards any point.
  if (v <= p.v_eps) return false;

  // Sample the unicycle arc (curvature w / v) up to the stopping distance.
  const double L = stopping_distance(v, p);
  const double k = w / v;

  for (double s = p.path_step;; s += p.path_step) {
    if (s > L) s = L;  // the last sample is exactly the stopping point

    // Robot centre after driving s along the arc from (0, 0) with heading 0.
    double x, y;
    if (std::fabs(k) < 1e-9) {
      x = s;
      y = 0.0;
    } else {
      x = std::sin(k * s) / k;
      y = (1.0 - std::cos(k * s)) / k;
    }

    for (const Point &pt : points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
      const double d = std::hypot(pt.x - x, pt.y - y);  // distance at path position s
      const double d0 = std::hypot(pt.x, pt.y);         // distance now
      // Risk only if the point enters the clearance while the robot approaches it: points behind or
      // beside the robot that it drives away from must not block motion.
      if (d < C && d < d0 - 1e-6) return true;
    }
    if (s >= L) break;
  }

  return false;
}

Decision SafetyCore::step(double now_sim, double now_steady) {
  const Params &p = params_;
  Decision d;  // v = w = 0: every early return below stops the robot

  auto from_future = [&](double t_sim) { return t_sim > now_sim + p.time_jump_tol; };
  if ((cmd_ && from_future(cmd_->t_sim)) || (odom_ && from_future(odom_->t_sim)) ||
      (scan_ && from_future(scan_->t_sim))) {
    reset();
    d.reason = Reason::TIME_JUMP;
    return d;
  }
  if (!cmd_ || !odom_ || !scan_) {
    d.reason = Reason::NOT_READY;
    return d;
  }
  if (now_sim - scan_->t_sim > p.sensor_timeout_sim) {
    d.reason = Reason::STALE_SCAN;
    return d;
  }
  if (now_sim - odom_->t_sim > p.sensor_timeout_sim) {
    d.reason = Reason::STALE_ODOM;
    return d;
  }
  if (latched_) {
    d.reason = Reason::STALE_CMD;
    return d;
  }
  const bool stale_by_sim = now_sim - cmd_->t_sim > p.cmd_timeout_sim;
  const bool stale_by_steady = now_steady - cmd_->t_steady > p.cmd_timeout_steady;
  if (stale_by_sim || stale_by_steady) {
    latched_ = true;
    latched_stamp_ = cmd_->t_sim;  // only a newer command releases the latch, see on_cmd()
    d.reason = Reason::STALE_CMD;
    return d;
  }

  const Cmd &c = *cmd_;

  if (!std::isfinite(c.v) || !std::isfinite(c.w)) {
    d.reason = Reason::NONFINITE;
    return d;
  }
  if (c.v < -p.limit_tol || c.v > p.v_max + p.limit_tol || std::fabs(c.w) > p.w_max + p.limit_tol) {
    d.reason = Reason::LIMIT;
    return d;
  }
  // The robot may still move faster than commanded, so the envelope uses the larger speed.
  const double v_eff = std::max({c.v, odom_->v, 0.0});
  if (collision_risk(v_eff, c.w, scan_->points, p)) {
    d.reason = Reason::COLLISION_RISK;
    return d;
  }

  d.v = c.v;
  d.w = c.w;
  d.reason = Reason::OK;
  return d;
}

}  // namespace capstone_safety
