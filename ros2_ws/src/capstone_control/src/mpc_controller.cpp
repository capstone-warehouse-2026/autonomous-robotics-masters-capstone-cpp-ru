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
  // TODO: names equal to the enumerators ("OK", "MAX_ITER", ...); no default case, return "UNKNOWN" after
  // the switch (same pattern as capstone_safety::to_string).
  (void)s;
  return "TODO";
}

MpcController::MpcController(const MpcParams &params) : params_(params) {
  // TODO: throw std::invalid_argument when
  //   - any double parameter is not finite;
  //   - horizon < 1, dt <= 0;
  //   - not 0 <= v_min < v_max; w_max, accel_max, decel_max, yaw_accel_max <= 0;
  //   - not 0 < v_ref <= v_max;
  //   - not 0 < robot_radius <= d_safe; lookahead <= 0, path_step <= 0, goal_zone < 0;
  //   - any weight < 0;
  //   - max_iterations < 0, max_solve_time < 0, fd_eps <= 0, tol < 0.
}

void MpcController::reset() {
  // TODO: forget the warm start and the last result.
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
  // Exact integration with constant v, w over dt, yaw1 = yaw + w * dt:
  //   |w| < 1e-9:  x += v * dt * cos(yaw),                  y += v * dt * sin(yaw)
  //   otherwise:   x += v / w * (sin(yaw1) - sin(yaw)),     y -= v / w * (cos(yaw1) - cos(yaw))
  // Keep yaw normalized.
  (void)u;
  (void)p;
  return {};
}

PathProjection MpcController::project_on_path(const std::vector<Point2D> &path, double x, double y) {
  // TODO: nearest point of the polyline to (x, y).
  //   For each segment i (path[i] -> path[i+1]) with non-zero length:
  //     t = dot(q - a, b - a) / |b - a|^2 clamped to [0, 1], nearest point a + t (b - a), distance to it.
  //   Keep the segment with the smallest distance and return
  //     distance:  that smallest distance,
  //     heading:   atan2 of that segment,
  //     remaining: (1 - t) * |b - a| + lengths of all later segments.
  //   No segment of non-zero length (the path collapsed to the goal): distance to the last point, heading 0,
  //   remaining 0.
  //   Hint: precompute the tail lengths from the end once, then one pass over the segments.
  (void)path;
  (void)x;
  (void)y;
  return {};
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
