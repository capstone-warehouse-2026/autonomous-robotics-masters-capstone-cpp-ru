#pragma once

// Method A of G4: receding-horizon nonlinear MPC (Model Predictive Control) for a differential drive.
//
// The controller works in the robot frame (base_link). make_problem() cuts the path ahead of the
// PathTracker projection (lookahead, sampled every path_step) and expresses it, the goal yaw and the
// obstacles relative to the robot; solve() optimizes over this LocalProblem; compute() is the Controller
// interface of docs/adr/0004 on top of both.
//
// Formulation (single shooting). Decision variables are the controls u_k = (v_k, w_k), k = 0..N-1, each
// held for dt. The poses x_k = (x, y, theta) follow from the unicycle model, integrated exactly over dt
// (an arc for w != 0, as in the safety monitor), starting at the robot (0, 0, 0), see rollout(). The cost is
//
//   J(u) = sum_{k=1..N} [ w_track * e_k^2 + w_heading * psi_k^2 ]                 tracking of the path
//        + sum_{k=0..N-1} [ w_speed * (v_k - vref_k)^2 + w_turn * w_k^2 ]         progress and effort
//        + sum_{k=0..N-1} [ w_dv * (v_k - v_{k-1})^2 + w_dw * (w_k - w_{k-1})^2 ] smoothness, u_{-1} = measured
//        + sum_{k=1..N} sum_obstacles w_obs * max(0, d_safe - d)^2               obstacles (soft)
//
// e_k is the distance from x_k to the path; psi_k is the heading error to the path tangent at the nearest
// point, except near the goal: if the path ends at the goal and less than goal_zone of path remains after
// x_k, psi_k is the error to the goal yaw and its weight is w_goal_heading (the final turn on the spot that
// the FollowPath success condition needs, 0.2 rad). vref_k = min(v_ref, sqrt(2 * decel_max * remaining_k))
// if the path ends at the goal (the robot stops there), otherwise v_ref; remaining_k is the path length from
// the nearest point of the pose before u_k to the end.
//
// Hard constraints, kept by project(): v_min <= v_k <= v_max, |w_k| <= w_max,
// -decel_max dt <= v_k - v_{k-1} <= accel_max dt, |w_k - w_{k-1}| <= yaw_accel_max dt, u_{-1} = measured.
// Box limits win if both cannot hold. The limits equal CommandLimits of ADR 0004, so the CommandLimiter
// after the controller only has to adapt the 0.1 s model step to the 0.05 s control period.
//
// Solver: projected gradient descent with backtracking line search; finite-difference gradient. It stops
// on convergence, after max_iterations or when max_solve_time is spent, and returns the best solution so
// far; the solution is shifted by one step and reused as the next warm start.
//
// No valid command (ADR 0004): invalid input, or a collision still predicted for the best solution (an
// obstacle closer than robot_radius to a predicted pose). compute() then returns std::nullopt and the node
// publishes zero; three such cycles in a row abort FollowPath with NO_VALID_CONTROL.

#include <optional>
#include <string>
#include <vector>

#include "capstone_control/controller.hpp"
#include "capstone_control/geometry.hpp"

namespace capstone_control {

struct MpcParams {
  // Horizon: N steps of dt = 2 s ahead, longer than the 0.5 s stopping time from full speed.
  int horizon = 20;
  double dt = 0.1;  // s

  // Robot limits, docs/adr/0004 (CommandLimits).
  double v_min = 0.0;          // m/s, no reversing
  double v_max = 0.5;          // m/s
  double w_max = 1.0;          // rad/s
  double accel_max = 0.5;      // m/s^2
  double decel_max = 1.0;      // m/s^2
  double yaw_accel_max = 2.0;  // rad/s^2

  // Reference and geometry.
  double v_ref = 0.45;         // m/s, cruise speed slightly below v_max
  double robot_radius = 0.25;  // m
  double d_safe = 0.45;        // m, obstacle penalty starts here (> safety monitor clearance 0.35)
  double lookahead = 3.0;      // m of path passed to the solver (>= v_max * horizon * dt)
  double path_step = 0.1;      // m between the sampled path points
  double goal_zone = 0.5;      // m of remaining path where the goal yaw replaces the path tangent

  // Cost weights.
  double w_track = 10.0;
  double w_heading = 1.0;
  double w_goal_heading = 5.0;
  double w_speed = 5.0;   // strong enough that slowing down is not cheaper than going around an obstacle
  double w_turn = 0.05;
  double w_dv = 1.0;
  double w_dw = 0.1;
  double w_obs = 200.0;

  // Solver.
  int max_iterations = 60;
  double max_solve_time = 0.025;  // s, half of the 50 ms control period
  double fd_eps = 1e-5;           // finite-difference step
  double tol = 1e-6;              // relative cost decrease that counts as converged
};

struct MpcControl {
  double v = 0.0;
  double w = 0.0;
};

// What the solver sees: everything in the robot frame (robot at the origin, heading +x).
struct LocalProblem {
  std::vector<Point2D> path;     // >= 2 points from the projection forwards, ordered towards the goal
  bool path_ends_at_goal = true;  // the last point is the goal (false: cut at the lookahead)
  double goal_yaw = 0.0;         // goal heading relative to the robot, rad; used if path_ends_at_goal
  std::vector<Point2D> obstacles;
  Command velocity;              // measured
};

enum class MpcStatus {
  OK,             // converged
  MAX_ITER,       // iteration limit reached, best solution used
  TIMEOUT,        // time budget spent, best solution so far used
  INFEASIBLE,     // collision predicted even for the best solution: no valid command
  INVALID_INPUT,  // fewer than 2 path points or NaN/Inf in the input: no valid command
};

std::string to_string(MpcStatus s);

struct MpcResult {
  std::optional<Command> command;  // u_0 of the solution; empty for INFEASIBLE and INVALID_INPUT
  MpcStatus status = MpcStatus::INVALID_INPUT;
  int iterations = 0;
  double cost = 0.0;
  double solve_time = 0.0;         // s, wall time of solve()
  std::vector<Pose2D> predicted;   // x_1..x_N of the solution (robot frame), for RViz and logs
};

// Nearest point of a polyline to a position.
struct PathProjection {
  double distance = 0.0;   // cross-track error e
  double heading = 0.0;    // direction of the path segment at the nearest point, rad
  double remaining = 0.0;  // path length from the nearest point to the end of the polyline
};

class MpcController : public Controller {
 public:
  // Throws std::invalid_argument for invalid parameters.
  explicit MpcController(const MpcParams &params = MpcParams());

  // Controller interface (docs/adr/0004): make_problem() + solve().
  void reset() override;
  std::optional<Command> compute(const ControlInput &input) override;

  // One optimization from the warm start.
  MpcResult solve(const LocalProblem &problem);

  const MpcParams &params() const { return params_; }
  const MpcResult &last_result() const { return last_; }

  // Building blocks, public for tests.
  static LocalProblem make_problem(const ControlInput &input, const MpcParams &p);
  static std::vector<Pose2D> rollout(const std::vector<MpcControl> &u, const MpcParams &p);
  static PathProjection project_on_path(const std::vector<Point2D> &path, double x, double y);
  static double cost(const std::vector<MpcControl> &u, const LocalProblem &problem, const MpcParams &p);
  static void project(std::vector<MpcControl> &u, const Command &measured, const MpcParams &p);

 private:
  MpcParams params_;
  std::vector<MpcControl> warm_;  // previous solution shifted by one step; empty after reset()
  MpcResult last_;
};

}  // namespace capstone_control
