#include "capstone_control/mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace capstone_control {

// TODO(G4): the whole file is a template. Each function lists what it must do; the formulation is in
// mpc_controller.hpp and the expected behaviour in test/test_mpc_controller.cpp. Reuse geometry.hpp:
// normalize_angle(), to_robot_frame(), Pose2D (yaw), Point2D, Command.

std::string to_string(MpcStatus s) {
  switch (s) {
    case MpcStatus::OK: return "OK";
    case MpcStatus::MAX_ITER: return "MAX_ITER";
    case MpcStatus::TIMEOUT: return "TIMEOUT";
    case MpcStatus::INFEASIBLE: return "INFEASIBLE";
    case MpcStatus::INVALID_INPUT: return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

MpcController::MpcController(const MpcParams &params) : params_(params) {
  const MpcParams &p = params_;
  const double all_doubles[] = {p.dt, p.v_min, p.v_max, p.w_max, p.accel_max, p.decel_max, p.yaw_accel_max, p.v_ref,
    p.robot_radius, p.d_safe, p.lookahead, p.path_step, p.goal_zone, p.w_track, p.w_heading, p.w_goal_heading, p.w_speed,
    p.w_turn, p.w_dv, p.w_dw, p.w_obs, p.max_solve_time, p.fd_eps, p.tol};
  for (double x : all_doubles) {
    if (!std::isfinite(x)) throw std::invalid_argument("MpcParams: non-finite parameter");
  }
  if (p.horizon < 1) throw std::invalid_argument("MpcParams: horizon parameter error");
  if (p.dt <= 0) throw std::invalid_argument("MpcParams: time parameter error");
  if (!(0 <= p.v_min && p.v_min < p.v_max)) throw std::invalid_argument("MpcParams: velocity parameter error");
  if (p.w_max <= 0) throw std::invalid_argument("MpcParams: omega parameter error");
  if (p.accel_max <= 0) throw std::invalid_argument("MpcParams: acceleration parameter error");
  if (p.decel_max <= 0) throw std::invalid_argument("MpcParams: decel parameter error");
  if (p.yaw_accel_max <= 0) throw std::invalid_argument("MpcParams: yaw_accel parameter error");
  if (!(0 < p.v_ref && p.v_ref <= p.v_max)) throw std::invalid_argument("MpcParams: v_ref parameter error");
  if (!(0 < p.robot_radius && p.robot_radius <= p.d_safe)) throw std::invalid_argument("MpcParams: robot_radius parameter error");
  if (p.lookahead <= 0) throw std::invalid_argument("MpcParams: lookahead parameter error");
  if (p.path_step <= 0) throw std::invalid_argument("MpcParams: path_step parameter error");
  if (p.goal_zone < 0) throw std::invalid_argument("MpcParams: goal_zone parameter error");
  if (p.w_dv < 0 || p.w_dw < 0 || p.w_goal_heading < 0 || p.w_heading < 0 ||  p.w_obs < 0 || p.w_speed < 0
    || p.w_track < 0 || p.w_turn < 0) throw std::invalid_argument("MpcParams: weight parameter error");
  if (p.max_iterations < 0) throw std::invalid_argument("MpcParams: max_iterations parameter error");
  if (p.max_solve_time < 0) throw std::invalid_argument("MpcParams: max_solve_time parameter error");
  if (p.fd_eps <= 0) throw std::invalid_argument("MpcParams: fd_eps parameter error");
  if (p.tol < 0) throw std::invalid_argument("MpcParams: tol parameter error");
}

void MpcController::reset() {
  warm_.clear();
  last_ = MpcResult{};
}

LocalProblem MpcController::make_problem(const ControlInput &input, const MpcParams &p) {
  // TODO: the robot-frame view of the input (input.path is non-null and already updated with input.pose).
  //   - velocity and obstacles: copied (obstacles are already in base_link);
  //   - goal_yaw = normalize_angle(goal().yaw - pose.yaw);
  //   - single-pose path (total_length() ~ 0): path = {(0, 0), goal in the robot frame}, ends at goal;
  //   - otherwise cut = min(lookahead, remaining_length()); path_ends_at_goal = remaining_length() <= lookahead;
  //     points point_ahead(0), point_ahead(path_step), ... up to point_ahead(cut) (add the end point if the
  //     last step does not land on it, and always keep >= 2 points), each through to_robot_frame(pose, ...).
  (void)input;
  (void)p;
  return {};
}

std::vector<Pose2D> MpcController::rollout(const std::vector<MpcControl> &u, const MpcParams &p) {
  // TODO: start at (0, 0, 0); for every control return the pose AFTER applying it for dt (u.size() poses).
  std::vector<Pose2D> poses;
  poses.reserve(u.size());
  Pose2D pose;
  for (const MpcControl &c : u) {
    const double yaw1 = pose.yaw + c.w * p.dt;
    if (std::abs(c.w) <= 1e-9) {
      pose.x += c.v * p.dt * std::cos(pose.yaw);
      pose.y += c.v * p.dt * std::sin(pose.yaw);
    }
    else {
      pose.x += (c.v/c.w)*(std::sin(yaw1) - std::sin(pose.yaw));
      pose.y -= (c.v/c.w)*(std::cos(yaw1) - std::cos(pose.yaw));
    }
    pose.yaw = normalize_angle(yaw1);
    poses.push_back(pose);
  }
  return poses;
}

PathProjection MpcController::project_on_path(const std::vector<Point2D> &path, double x, double y) {
  // TODO: nearest point of the polyline to (x, y).
  const std::size_t n = path.size();
  if (!n) throw std::invalid_argument("empty path error");
  std::vector<double> tail(n, 0.0);
  for (std::size_t i = n-1; i-- > 0;) {
    tail[i] = tail[i+1] + std::hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
  }

  PathProjection best;
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;

  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Point2D &a = path[i];
    const Point2D &b = path[i+1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) continue;
    const double t = std::clamp(((x - a.x)*dx + (y - a.y)*dy) / len2, 0.0, 1.0);
    const double px = a.x + t * dx;
    const double py = a.y + t * dy;
    const double d = std::hypot(px - x, py - y);
    if (d < best_distance) {
      best_distance = d;
      best.distance = d;
      best.heading = std::atan2(dy, dx);
      best.remaining = (1-t) * std::sqrt(len2) + tail[i+1];
      found = true;
    }
  }
  if (!found) {
    best.distance = std::hypot(x - path[n-1].x, y - path[n-1].y);
    best.heading = 0;
    best.remaining = 0;
  }
  return best;
}

double MpcController::cost(const std::vector<MpcControl> &u, const LocalProblem &problem, const MpcParams &p) {
  // TODO: J(u) from mpc_controller.hpp.
  //   poses = rollout(u, p). For k = 0..N-1:
  //     - pose before u_k: the origin for k = 0, otherwise poses[k-1]; its projection gives remaining_k;
  //       vref_k = path_ends_at_goal ? min(v_ref, sqrt(2 decel_max remaining_k)) : v_ref
  //     - speed and effort:  w_speed (v_k - vref_k)^2 + w_turn w_k^2
  //     - smoothness:        w_dv (v_k - v_{k-1})^2 + w_dw (w_k - w_{k-1})^2, u_{-1} = problem.velocity
  //     - pose after u_k = poses[k], its projection pr:
  //         w_track pr.distance^2
  //         + (path_ends_at_goal && pr.remaining < goal_zone ? w_goal_heading * err(yaw, goal_yaw)^2
  //                                                          : w_heading * err(yaw, pr.heading)^2)
  //       err = normalize_angle(difference)
  //     - obstacles: for each obstacle at distance d < d_safe from poses[k]: w_obs (d_safe - d)^2
  (void)u;
  (void)problem;
  (void)p;
  return 0.0;
}

void MpcController::project(std::vector<MpcControl> &u, const Command &measured, const MpcParams &p) {
  // TODO: make u feasible in place, step by step, with (v_prev, w_prev) = measured for k = 0:
  //   lo = max(v_min, v_prev - decel_max dt), hi = min(v_max, v_prev + accel_max dt)
  //   if lo <= hi: v_k = clamp(v_k, lo, hi)
  //   else (measured speed already outside the box): v_k = v_prev > v_max ? v_max : v_min
  //   the same for w with yaw_accel_max and [-w_max, w_max]; then (v_prev, w_prev) = u_k.
  (void)u;
  (void)measured;
  (void)p;
}

std::optional<Command> MpcController::compute(const ControlInput &input) {
  // TODO: no path (input.path null or empty) -> clear last_ and return std::nullopt;
  // otherwise last_ = solve(make_problem(input, params_)) and return last_.command.
  (void)input;
  return std::nullopt;
}

MpcResult MpcController::solve(const LocalProblem &problem) {
  MpcResult r;
  // TODO, in this order:
  //  1. Validity: path.size() >= 2 and every number in the problem finite. Otherwise status INVALID_INPUT,
  //     no command, clear the warm start, return.
  //  2. Initial guess u: the warm start if it has `horizon` elements, otherwise v_ref and w = 0 everywhere.
  //     project(u, problem.velocity); J = cost(u).
  //  3. Projected gradient with backtracking, at most max_iterations iterations:
  //       - before every iteration: if the time since the start of solve() >= max_solve_time -> TIMEOUT, stop;
  //       - gradient by forward differences: g_i = (cost(u + fd_eps e_i) - J) / fd_eps for all 2N variables;
  //       - line search: u_new = project(u - step * g); accept if cost(u_new) < J, else halve step
  //         (up to ~20 times); start with step = 0.05 and double it after an accepted step;
  //       - no decrease found -> converged (OK); decrease <= tol * (1 + J) -> accept it and converged (OK);
  //       - loop ended by max_iterations -> MAX_ITER.
  //  4. Result: command = u_0, iterations, cost, predicted = rollout(u).
  //  5. Infeasible: if any predicted pose is closer than robot_radius to any obstacle -> INFEASIBLE and
  //     no command (the node publishes zero, ADR 0004).
  //  6. Warm start for the next cycle: u shifted by one step, the last element repeated.
  //  7. solve_time = wall time of the call (std::chrono::steady_clock).
  (void)problem;
  return r;
}

}  // namespace capstone_control
