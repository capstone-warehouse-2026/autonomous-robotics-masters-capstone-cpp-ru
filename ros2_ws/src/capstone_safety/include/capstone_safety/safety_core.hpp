#pragma once

// Decision logic of the G4 safety monitor without ROS, so that it is tested deterministically.
// SafetyMonitorNode converts messages into these structures, calls step() from its 20 Hz timer and
// publishes the result on /cmd_vel. Fault reactions: docs/INTERFACES.md, fault table.
//
// Times are in seconds: t_sim is header.stamp (simulation time), t_steady is the receive time on the
// steady clock. Scan points are already in base_link (robot centre at the origin, x forward, y left)
// and free of NaN/Inf.

#include <optional>
#include <string>
#include <vector>

namespace capstone_safety {

struct Params {
  // Command limits, docs/PROJECT.md. A command outside [-limit_tol, v_max + limit_tol] or with
  // |w| > w_max + limit_tol is an invalid controller output and stops the robot; it is not clamped.
  double v_max = 0.6;
  double w_max = 1.2;
  double limit_tol = 0.02;

  // Stopping model: d_stop(v) = v * t_latency + v^2 / (2 * a_brake).
  double a_brake = 0.5;     // m/s^2, to be justified by a braking measurement in the simulator
  double t_latency = 0.15;  // s: monitor period + driver delay + scan age

  // The robot is a disc of robot_radius; along the stopping path no obstacle may come closer than
  // robot_radius + margin.
  double robot_radius = 0.25;
  double margin = 0.10;
  double path_step = 0.02;  // m, sampling step of the stopping path
  double v_eps = 1e-3;      // m/s, below this the robot is not translating

  // Freshness, docs/INTERFACES.md, fault table.
  double cmd_timeout_sim = 0.15;
  double cmd_timeout_steady = 0.30;
  double sensor_timeout_sim = 0.30;
  double time_jump_tol = 0.05;  // s a stamp may lie in the future before it counts as a time reset
};

struct Stamped {
  double t_sim = 0.0;
  double t_steady = 0.0;
};

struct Cmd : Stamped {
  double v = 0.0;
  double w = 0.0;
};

struct Odom : Stamped {
  double v = 0.0;  // measured forward speed in base_link
};

struct Point {
  double x = 0.0;
  double y = 0.0;
};

struct Scan : Stamped {
  std::vector<Point> points;
};

// The order of the enumerators is the order of the checks in step(): the first one that fails wins.
enum class Reason {
  OK,
  TIME_JUMP,       // time went backwards (simulation reset); the state is cleared
  NOT_READY,       // no command, odom or scan yet
  STALE_SCAN,
  STALE_ODOM,
  STALE_CMD,       // also while latched: an old command is never resumed
  NONFINITE,       // NaN/Inf in the command
  LIMIT,           // command outside the limits
  COLLISION_RISK,  // obstacle inside the stopping envelope
};

std::string to_string(Reason r);

struct Decision {
  double v = 0.0;
  double w = 0.0;
  Reason reason = Reason::NOT_READY;
  bool intervened() const { return reason != Reason::OK; }
};

class SafetyCore {
 public:
  // Throws std::invalid_argument for invalid parameters.
  explicit SafetyCore(const Params &params = Params());

  void on_cmd(const Cmd &cmd);
  void on_odom(const Odom &odom);
  void on_scan(const Scan &scan);

  // Called by the 20 Hz timer; returns the command to publish on /cmd_vel. Any reason other than OK
  // gives v = w = 0.
  Decision step(double now_sim, double now_steady);

  // Clears all state, as on node start.
  void reset();

  bool latched() const { return latched_; }
  const Params &params() const { return params_; }

  // Static so that the geometry is tested separately from the timing logic.
  static double stopping_distance(double v, const Params &p);
  static bool collision_risk(double v, double w, const std::vector<Point> &points, const Params &p);

 private:
  Params params_;
  std::optional<Cmd> cmd_;
  std::optional<Odom> odom_;
  std::optional<Scan> scan_;

  bool latched_ = false;
  double latched_stamp_ = 0.0;  // t_sim of the command that triggered STALE_CMD
};

}  // namespace capstone_safety
