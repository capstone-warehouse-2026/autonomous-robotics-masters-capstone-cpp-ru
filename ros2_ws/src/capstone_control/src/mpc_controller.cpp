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
  const std::vector<Pose2D> poses = rollout(u, p);
  double J = 0.0;
  double v_prev = problem.velocity.v;
  double w_prev = problem.velocity.w;

  for (std::size_t k = 0; k < u.size(); ++k) {
    const Pose2D before = (k == 0) ? Pose2D{} : poses[k-1];
    const Pose2D &after = poses[k];
    // Слежение за путем
    const PathProjection pr = project_on_path(problem.path, after.x, after.y);
    J += p.w_track * pr.distance * pr.distance;
    if (problem.path_ends_at_goal && pr.remaining < p.goal_zone) {
      const double heading_err = normalize_angle(after.yaw - problem.goal_yaw);
      J += p.w_goal_heading * heading_err * heading_err;
    } else {
      const double heading_err = normalize_angle(after.yaw - pr.heading);
      J += p.w_heading * heading_err * heading_err;
    }
    // Препятствия
    for (const Point2D &o : problem.obstacles) {
      const double d = std::hypot(after.x - o.x, after.y - o.y);
      if (d < p.d_safe) {
        const double gap = p.d_safe - d;
        J += p.w_obs * gap * gap;
      }
    }
    // Скорость и поворот
    double vref = p.v_ref;
    if (problem.path_ends_at_goal) {
      const double remaining = project_on_path(problem.path, before.x, before.y).remaining;
      vref = std::min(p.v_ref, std::sqrt(2.0 * p.decel_max * remaining));
    }
    J += p.w_speed * (u[k].v - vref) * (u[k].v - vref);
    J += p.w_turn * u[k].w * u[k].w;
    //Плавность
    J += p.w_dv * (u[k].v - v_prev) * (u[k].v - v_prev);
    J += p.w_dw * (u[k].w - w_prev) * (u[k].w - w_prev);


    v_prev = u[k].v;
    w_prev = u[k].w;
  }
  return J;
}

void MpcController::project(std::vector<MpcControl> &u, const Command &measured, const MpcParams &p) {
  // TODO: make u feasible in place, step by step, with (v_prev, w_prev) = measured for k = 0:
  if (u.empty()) return;
  const std::size_t n = u.size();
  const double lo0 = std::max(p.v_min, measured.v - p.decel_max * p.dt);
  const double hi0 = std::min(p.v_max, measured.v + p.accel_max * p.dt);
  if (lo0 <= hi0) u[0].v = std::clamp(u[0].v, lo0, hi0);
  else u[0].v = (measured.v > p.v_max) ? p.v_max : p.v_min;
  for (std::size_t i = 1; i < n; ++i) {
    double lo = std::max(p.v_min, u[i-1].v - p.decel_max * p.dt);
    double hi = std::min(p.v_max, u[i-1].v + p.accel_max * p.dt);
    if (lo <= hi) u[i].v = std::clamp(u[i].v, lo, hi);
    else u[i].v = (u[i-1].v > p.v_max) ? p.v_max : p.v_min;
  }

  const double lo0w = std::max(-p.w_max, measured.w - p.yaw_accel_max * p.dt);
  const double hi0w = std::min(p.w_max, measured.w + p.yaw_accel_max * p.dt);
  if (lo0w <= hi0w) u[0].w = std::clamp(u[0].w, lo0w, hi0w);
  else u[0].w = (measured.w > p.w_max) ? p.w_max : -p.w_max;
  for (std::size_t i = 1; i < n; ++i) {
    double lo = std::max(-p.w_max, u[i-1].w - p.yaw_accel_max * p.dt);
    double hi = std::min(p.w_max, u[i-1].w + p.yaw_accel_max * p.dt);
    if (lo <= hi) u[i].w = std::clamp(u[i].w, lo, hi);
    else u[i].w = (u[i-1].w > p.w_max) ? p.w_max : -p.w_max;
  }
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
