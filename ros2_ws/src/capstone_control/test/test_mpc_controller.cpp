// Unit tests of MpcController (method A): model, path projection, constraints, cost, the solver, the
// conversion from ControlInput, and closed-loop runs through the same CommandLimiter and GoalChecker as
// the node (as test_closed_loop.cpp does for pure pursuit). No ROS; the solver tests use a large time
// budget so that they are deterministic on any machine.

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include "capstone_control/checkers.hpp"
#include "capstone_control/command_limiter.hpp"
#include "capstone_control/mpc_controller.hpp"

namespace {

using namespace capstone_control;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// Robot-frame straight path along x from x0 to x1 at lateral offset y, a point every 10 cm.
std::vector<Point2D> straight_points(double x0, double x1, double y = 0.0) {
  std::vector<Point2D> path;
  const int n = static_cast<int>(std::round((x1 - x0) / 0.1));
  for (int i = 0; i <= n; ++i) path.push_back({x0 + 0.1 * i, y});
  return path;
}

LocalProblem problem(std::vector<Point2D> path, double v = 0.0, double w = 0.0, bool ends_at_goal = true,
                     double goal_yaw = 0.0) {
  LocalProblem p;
  p.path = std::move(path);
  p.path_ends_at_goal = ends_at_goal;
  p.goal_yaw = goal_yaw;
  p.velocity = Command{v, w};
  return p;
}

// Deterministic solver settings: convergence or the iteration limit ends the solve, never the clock.
MpcParams no_timeout() {
  MpcParams p;
  p.max_solve_time = 10.0;
  p.max_iterations = 200;
  return p;
}

std::vector<MpcControl> constant(double v, double w, int n) { return std::vector<MpcControl>(n, MpcControl{v, w}); }

// ---------------------------------------------------------------- parameters

TEST(MpcParamsTest, RejectsInvalid) {
  MpcParams p;
  p.horizon = 0;
  EXPECT_THROW(MpcController{p}, std::invalid_argument);
  p = MpcParams{};
  p.dt = 0.0;
  EXPECT_THROW(MpcController{p}, std::invalid_argument);
  p = MpcParams{};
  p.v_ref = 0.6;  // above v_max
  EXPECT_THROW(MpcController{p}, std::invalid_argument);
  p = MpcParams{};
  p.d_safe = 0.1;  // below robot_radius
  EXPECT_THROW(MpcController{p}, std::invalid_argument);
  p = MpcParams{};
  p.decel_max = 0.0;
  EXPECT_THROW(MpcController{p}, std::invalid_argument);
  p = MpcParams{};
  p.w_obs = kNan;
  EXPECT_THROW(MpcController{p}, std::invalid_argument);
}

TEST(MpcParamsTest, LimitsMatchCommandLimiter) {
  // ADR 0004: MPC and the shared limiter use the same actuation limits.
  const MpcParams p;
  const CommandLimits l;
  EXPECT_DOUBLE_EQ(p.v_max, l.max_speed);
  EXPECT_DOUBLE_EQ(p.w_max, l.max_yaw_rate);
  EXPECT_DOUBLE_EQ(p.accel_max, l.max_accel);
  EXPECT_DOUBLE_EQ(p.decel_max, l.max_decel);
  EXPECT_DOUBLE_EQ(p.yaw_accel_max, l.max_yaw_accel);
}

TEST(MpcParamsTest, StatusNames) {
  EXPECT_EQ(to_string(MpcStatus::OK), "OK");
  EXPECT_EQ(to_string(MpcStatus::INFEASIBLE), "INFEASIBLE");
}

// ---------------------------------------------------------------- model

TEST(Rollout, StraightLine) {
  const MpcParams p;  // dt = 0.1
  const auto poses = MpcController::rollout(constant(0.5, 0.0, 10), p);
  ASSERT_EQ(poses.size(), 10u);
  EXPECT_NEAR(poses.back().x, 0.5, 1e-9);  // 10 steps * 0.1 s * 0.5 m/s
  EXPECT_NEAR(poses.back().y, 0.0, 1e-9);
  EXPECT_NEAR(poses.back().yaw, 0.0, 1e-9);
}

TEST(Rollout, ArcMatchesCircle) {
  const MpcParams p;
  const double v = 0.4, w = 0.8, T = 1.0;  // 10 steps
  const auto poses = MpcController::rollout(constant(v, w, 10), p);
  ASSERT_EQ(poses.size(), 10u);
  // Exact unicycle with constant v, w: circle of radius v / w around (0, v / w).
  EXPECT_NEAR(poses.back().x, v / w * std::sin(w * T), 1e-9);
  EXPECT_NEAR(poses.back().y, v / w * (1.0 - std::cos(w * T)), 1e-9);
  EXPECT_NEAR(poses.back().yaw, w * T, 1e-9);
}

TEST(Rollout, TurnInPlaceKeepsPosition) {
  const MpcParams p;
  const auto poses = MpcController::rollout(constant(0.0, 1.0, 5), p);
  ASSERT_EQ(poses.size(), 5u);
  EXPECT_NEAR(poses.back().x, 0.0, 1e-12);
  EXPECT_NEAR(poses.back().y, 0.0, 1e-12);
  EXPECT_NEAR(poses.back().yaw, 0.5, 1e-9);
}

// ---------------------------------------------------------------- path projection

TEST(ProjectOnPath, PointBesideStraightPath) {
  const auto pr = MpcController::project_on_path(straight_points(0.0, 2.0), 0.5, 0.3);
  EXPECT_NEAR(pr.distance, 0.3, 1e-9);
  EXPECT_NEAR(pr.heading, 0.0, 1e-9);
  EXPECT_NEAR(pr.remaining, 1.5, 1e-9);  // from x = 0.5 to the end at x = 2.0
}

TEST(ProjectOnPath, BeyondEndClampsToLastPoint) {
  const auto pr = MpcController::project_on_path(straight_points(0.0, 1.0), 1.5, 0.0);
  EXPECT_NEAR(pr.distance, 0.5, 1e-9);
  EXPECT_NEAR(pr.remaining, 0.0, 1e-9);
}

TEST(ProjectOnPath, HeadingOfDiagonalSegment) {
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 1.0}};
  const auto pr = MpcController::project_on_path(path, 0.5, 0.5);
  EXPECT_NEAR(pr.distance, 0.0, 1e-9);
  EXPECT_NEAR(pr.heading, kPi / 4, 1e-9);
}

TEST(ProjectOnPath, DegeneratePathIsAPoint) {
  // At the goal the cut path collapses to one point twice: distance to it, nothing remaining.
  const std::vector<Point2D> path{{0.3, 0.4}, {0.3, 0.4}};
  const auto pr = MpcController::project_on_path(path, 0.0, 0.0);
  EXPECT_NEAR(pr.distance, 0.5, 1e-9);
  EXPECT_NEAR(pr.remaining, 0.0, 1e-9);
}

// ---------------------------------------------------------------- constraints

TEST(Project, ClampsToBoxAndRate) {
  const MpcParams p;  // accel 0.05, yaw accel 0.2 per 0.1 s step
  auto u = constant(1.0, 2.0, 20);
  MpcController::project(u, Command{0.0, 0.0}, p);
  for (int k = 0; k < 20; ++k) {
    EXPECT_NEAR(u[k].v, std::min(0.5, 0.05 * (k + 1)), 1e-12) << "k=" << k;  // ramps up, then v_max
    EXPECT_NEAR(u[k].w, std::min(1.0, 0.2 * (k + 1)), 1e-12) << "k=" << k;
  }
}

TEST(Project, BrakesFasterThanItAccelerates) {
  const MpcParams p;  // decel 0.1 per step
  auto u = constant(0.0, 0.0, 6);
  MpcController::project(u, Command{0.5, 0.0}, p);
  for (int k = 0; k < 6; ++k) EXPECT_NEAR(u[k].v, std::max(0.0, 0.5 - 0.1 * (k + 1)), 1e-12) << "k=" << k;
}

TEST(Project, NoReversing) {
  const MpcParams p;
  auto u = constant(-0.3, 0.0, 5);
  MpcController::project(u, Command{0.0, 0.0}, p);
  for (const auto &c : u) EXPECT_DOUBLE_EQ(c.v, 0.0);
}

TEST(Project, BoxWinsWhenMeasuredSpeedIsOutside) {
  const MpcParams p;
  auto u = constant(0.7, 0.0, 3);
  MpcController::project(u, Command{0.8, 0.0}, p);  // odom above v_max + decel step
  for (const auto &c : u) EXPECT_DOUBLE_EQ(c.v, 0.5);
}

// ---------------------------------------------------------------- cost

TEST(Cost, OnPathIsCheaperThanOffPath) {
  const MpcParams p;
  const auto u = constant(0.45, 0.0, p.horizon);
  const double on = MpcController::cost(u, problem(straight_points(0.0, 5.0), 0.45), p);
  const double off = MpcController::cost(u, problem(straight_points(0.0, 5.0, 0.5), 0.45), p);
  EXPECT_LT(on, off);
}

TEST(Cost, ObstacleOnTrajectoryIsPenalized) {
  const MpcParams p;
  const auto u = constant(0.45, 0.0, p.horizon);
  auto lp = problem(straight_points(0.0, 5.0), 0.45);
  const double free = MpcController::cost(u, lp, p);
  lp.obstacles.push_back({0.6, 0.0});
  EXPECT_GT(MpcController::cost(u, lp, p), free + 1.0);
}

TEST(Cost, StoppingAtGoalIsCheaperThanOvershooting) {
  const MpcParams p;
  const auto lp = problem(straight_points(0.0, 0.3), 0.0);
  EXPECT_LT(MpcController::cost(constant(0.0, 0.0, p.horizon), lp, p),
            MpcController::cost(constant(0.45, 0.0, p.horizon), lp, p));
}

TEST(Cost, GoalYawMattersOnlyNearTheGoal) {
  const MpcParams p;
  const auto turn = constant(0.0, 0.5, p.horizon);  // turning on the spot towards +1 rad
  const auto stay = constant(0.0, 0.0, p.horizon);
  // At the goal with the goal yaw +1 rad: turning is cheaper.
  const auto at_goal = problem({{0.0, 0.0}, {0.0, 0.0}}, 0.0, 0.0, true, 1.0);
  EXPECT_LT(MpcController::cost(turn, at_goal, p), MpcController::cost(stay, at_goal, p));
  // Far from the goal the same goal yaw is ignored: keeping the path heading is cheaper.
  const auto far = problem(straight_points(0.0, 5.0), 0.0, 0.0, true, 1.0);
  EXPECT_GT(MpcController::cost(turn, far, p), MpcController::cost(stay, far, p));
}

// ---------------------------------------------------------------- solver

TEST(Solve, DrivesForwardOnStraightPath) {
  MpcController mpc(no_timeout());
  const auto r = mpc.solve(problem(straight_points(0.0, 5.0), 0.3));
  EXPECT_TRUE(r.status == MpcStatus::OK || r.status == MpcStatus::MAX_ITER) << to_string(r.status);
  ASSERT_TRUE(r.command.has_value());
  EXPECT_NEAR(r.command->v, 0.35, 1e-6);  // accelerates as fast as allowed: 0.3 + accel_max * dt
  EXPECT_NEAR(r.command->w, 0.0, 1e-3);
  ASSERT_EQ(r.predicted.size(), static_cast<std::size_t>(mpc.params().horizon));
}

TEST(Solve, TurnsTowardsPathOnTheLeft) {
  MpcController mpc(no_timeout());
  const auto r = mpc.solve(problem(straight_points(0.0, 5.0, 0.4), 0.3));
  ASSERT_TRUE(r.command.has_value());
  EXPECT_GT(r.command->w, 0.05);
}

TEST(Solve, TurnsTowardsPathOnTheRight) {
  MpcController mpc(no_timeout());
  const auto r = mpc.solve(problem(straight_points(0.0, 5.0, -0.4), 0.3));
  ASSERT_TRUE(r.command.has_value());
  EXPECT_LT(r.command->w, -0.05);
}

TEST(Solve, KeepsAllConstraints) {
  MpcController mpc(no_timeout());
  const double v0 = 0.45, w0 = -0.3;
  const auto r = mpc.solve(problem(straight_points(0.0, 3.0, 1.0), v0, w0));
  ASSERT_TRUE(r.command.has_value());
  const auto &p = mpc.params();
  EXPECT_GE(r.command->v, p.v_min);
  EXPECT_LE(r.command->v, p.v_max);
  EXPECT_LE(r.command->v - v0, p.accel_max * p.dt + 1e-9);
  EXPECT_GE(r.command->v - v0, -p.decel_max * p.dt - 1e-9);
  EXPECT_LE(std::fabs(r.command->w - w0), p.yaw_accel_max * p.dt + 1e-9);
  EXPECT_LE(std::fabs(r.command->w), p.w_max);
}

TEST(Solve, SlowsDownBeforeGoal) {
  MpcController mpc(no_timeout());
  const auto r = mpc.solve(problem(straight_points(0.0, 0.2), 0.45));  // goal 0.2 m ahead at 0.45 m/s
  ASSERT_TRUE(r.command.has_value());
  EXPECT_LT(r.command->v, 0.45);
}

TEST(Solve, KeepsCruisingWhenPathContinues) {
  MpcController mpc(no_timeout());
  // The segment was cut at the lookahead (longer than the horizon distance): no reason to slow down.
  const auto r = mpc.solve(problem(straight_points(0.0, 1.5), 0.45, 0.0, /*ends_at_goal=*/false));
  ASSERT_TRUE(r.command.has_value());
  EXPECT_GE(r.command->v, 0.45 - 1e-9);
}

TEST(Solve, TurnsOnTheSpotToGoalYaw) {
  MpcController mpc(no_timeout());
  const auto r = mpc.solve(problem({{0.0, 0.0}, {0.0, 0.0}}, 0.0, 0.0, true, 1.0));
  ASSERT_TRUE(r.command.has_value());
  EXPECT_GT(r.command->w, 0.05);
  EXPECT_LT(r.command->v, 0.05);
}

TEST(Solve, AvoidsObstacleOnPath) {
  MpcController mpc(no_timeout());
  auto lp = problem(straight_points(0.0, 5.0), 0.4);
  lp.obstacles.push_back({1.2, 0.0});
  const auto r = mpc.solve(lp);
  ASSERT_NE(r.status, MpcStatus::INFEASIBLE);
  double min_d = 1e9;
  for (const auto &x : r.predicted) min_d = std::min(min_d, std::hypot(1.2 - x.x, 0.0 - x.y));
  EXPECT_GE(min_d, mpc.params().robot_radius) << "predicted trajectory must not hit the obstacle";
}

TEST(Solve, UnavoidableCollisionHasNoValidCommand) {
  MpcController mpc(no_timeout());
  auto lp = problem(straight_points(0.0, 5.0), 0.5);
  for (int i = -20; i <= 20; ++i) lp.obstacles.push_back({0.3, 0.1 * i});  // wall 5 cm from the bumper
  const auto r = mpc.solve(lp);
  EXPECT_EQ(r.status, MpcStatus::INFEASIBLE);
  EXPECT_FALSE(r.command.has_value());
}

TEST(Solve, ZeroTimeBudgetTimesOutWithFeasibleCommand) {
  MpcParams p;
  p.max_solve_time = 0.0;
  MpcController mpc(p);
  const auto r = mpc.solve(problem(straight_points(0.0, 5.0), 0.2));
  EXPECT_EQ(r.status, MpcStatus::TIMEOUT);
  EXPECT_EQ(r.iterations, 0);
  ASSERT_TRUE(r.command.has_value());
  EXPECT_LE(std::fabs(r.command->v - 0.2), p.accel_max * p.dt + 1e-9);  // warm start was projected
}

TEST(Solve, InvalidInputHasNoValidCommand) {
  MpcController mpc(no_timeout());
  EXPECT_EQ(mpc.solve(problem({{0.0, 0.0}}, 0.4)).status, MpcStatus::INVALID_INPUT);  // a single point

  auto lp = problem(straight_points(0.0, 5.0), 0.4);
  lp.obstacles.push_back({kNan, 0.0});
  auto r = mpc.solve(lp);
  EXPECT_EQ(r.status, MpcStatus::INVALID_INPUT);
  EXPECT_FALSE(r.command.has_value());

  EXPECT_EQ(mpc.solve(problem(straight_points(0.0, 5.0), kNan)).status, MpcStatus::INVALID_INPUT);
}

TEST(Solve, DeterministicAfterReset) {
  MpcController mpc(no_timeout());
  const auto lp = problem(straight_points(0.0, 5.0, 0.3), 0.3, 0.1);
  const auto a = mpc.solve(lp);
  mpc.reset();
  const auto b = mpc.solve(lp);
  ASSERT_TRUE(a.command && b.command);
  EXPECT_DOUBLE_EQ(a.command->v, b.command->v);
  EXPECT_DOUBLE_EQ(a.command->w, b.command->w);
  EXPECT_EQ(a.iterations, b.iterations);
}

// ---------------------------------------------------------------- ControlInput -> LocalProblem

Path straight_map(double x0, double x1, double y = 0.0) {
  Path p;
  for (double x = x0; x <= x1 + 1e-9; x += 0.05) p.push_back({x, y, 0.0});
  return p;
}

TEST(MakeProblem, PathInRobotFrame) {
  PathTracker tracker(straight_map(0.0, 5.0, 1.0), 1.5);
  const Pose2D robot{1.0, 1.0, kPi / 2};  // on the path, facing +y (the path runs to the robot's right)
  tracker.update(robot);
  ControlInput in;
  in.pose = robot;
  in.path = &tracker;
  in.velocity = Command{0.2, 0.1};
  in.obstacles = {{1.0, 0.5}};
  const auto lp = MpcController::make_problem(in, MpcParams{});
  ASSERT_EQ(lp.path.size(), 31u);  // 3 m lookahead every 0.1 m, both ends included
  EXPECT_NEAR(lp.path.front().x, 0.0, 1e-9);
  EXPECT_NEAR(lp.path.front().y, 0.0, 1e-9);
  EXPECT_NEAR(lp.path.back().x, 0.0, 1e-9);  // 3 m along world +x = 3 m to the robot's right
  EXPECT_NEAR(lp.path.back().y, -3.0, 1e-9);
  EXPECT_FALSE(lp.path_ends_at_goal);  // 4 m remain, more than the lookahead
  EXPECT_NEAR(lp.goal_yaw, -kPi / 2, 1e-9);
  ASSERT_EQ(lp.obstacles.size(), 1u);  // obstacles are already in base_link: passed through
  EXPECT_DOUBLE_EQ(lp.velocity.v, 0.2);
}

TEST(MakeProblem, ShortRemainderEndsAtGoal) {
  PathTracker tracker(straight_map(0.0, 1.0), 1.5);
  const Pose2D robot{0.75, 0.0, 0.0};
  tracker.update(robot);
  ControlInput in;
  in.pose = robot;
  in.path = &tracker;
  const auto lp = MpcController::make_problem(in, MpcParams{});
  EXPECT_TRUE(lp.path_ends_at_goal);
  ASSERT_GE(lp.path.size(), 2u);
  EXPECT_NEAR(lp.path.back().x, 0.25, 1e-9);  // the goal, 0.25 m ahead
}

TEST(MakeProblem, SinglePoseGoesStraightToIt) {
  PathTracker tracker(Path{{2.0, 0.0, 1.0}}, 1.5);
  const Pose2D robot{0.0, 0.0, 0.0};
  tracker.update(robot);
  ControlInput in;
  in.pose = robot;
  in.path = &tracker;
  const auto lp = MpcController::make_problem(in, MpcParams{});
  ASSERT_EQ(lp.path.size(), 2u);
  EXPECT_NEAR(lp.path[1].x, 2.0, 1e-9);
  EXPECT_NEAR(lp.goal_yaw, 1.0, 1e-9);
}

TEST(Compute, WithoutPathHasNoCommand) {
  MpcController mpc(no_timeout());
  ControlInput in;  // in.path == nullptr
  EXPECT_FALSE(mpc.compute(in).has_value());
}

// ---------------------------------------------------------------- closed loop

struct RunResult {
  bool reached{false};
  bool stuck{false};
  bool collided{false};
  double max_cross_track{0.0};
  double min_clearance{1e9};
  int no_command_cycles{0};
};

// The same loop as test_closed_loop.cpp: 20 Hz, CommandLimiter, GoalChecker, ProgressChecker, ideal
// unicycle; obstacles are world points converted to base_link every cycle.
RunResult run(const Path &path, Pose2D pose, double timeout, const std::vector<Point2D> &world_obstacles = {}) {
  const double dt = 0.05;
  MpcParams params = no_timeout();
  params.max_iterations = 60;
  MpcController controller(params);
  CommandLimiter limiter;
  GoalChecker goal_checker;
  ProgressChecker progress;
  PathTracker tracker(path, 1.5);
  Command last{}, measured{};
  RunResult r;
  progress.reset(pose, 0.0);
  for (double t = 0.0; t < timeout; t += dt) {
    tracker.update(pose);
    r.max_cross_track = std::max(r.max_cross_track, std::abs(tracker.cross_track_error()));
    for (const auto &o : world_obstacles) {
      const double d = std::hypot(o.x - pose.x, o.y - pose.y);
      r.min_clearance = std::min(r.min_clearance, d);
      r.collided = r.collided || d < 0.25;
    }
    if (goal_checker.reached(pose, tracker.goal(), measured)) {
      r.reached = true;
      break;
    }
    if (!progress.check(pose, t)) {
      r.stuck = true;
      break;
    }
    ControlInput in;
    in.pose = pose;
    in.velocity = measured;
    in.path = &tracker;
    in.dt = dt;
    for (const auto &o : world_obstacles) in.obstacles.push_back(to_robot_frame(pose, o.x, o.y));
    const auto desired = controller.compute(in);
    if (!desired) ++r.no_command_cycles;
    last = limiter.limit(desired.value_or(Command{}), last, dt);
    measured = last;
    pose.x += measured.v * std::cos(pose.yaw) * dt;
    pose.y += measured.v * std::sin(pose.yaw) * dt;
    pose.yaw = normalize_angle(pose.yaw + measured.w * dt);
  }
  return r;
}

Path quarter_circle(double radius) {
  Path p;
  for (double a = 0.0; a <= kPi / 2 + 1e-9; a += 0.02) {
    p.push_back({radius * std::sin(a), radius * (1.0 - std::cos(a)), a});
  }
  return p;
}

Path l_shape() {
  Path p;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.05) p.push_back({x, 0.0, 0.0});
  for (double y = 0.05; y <= 3.0 + 1e-9; y += 0.05) p.push_back({3.0, y, kPi / 2});
  return p;
}

TEST(MpcClosedLoop, StraightPath) {
  const auto r = run(straight_map(0.0, 4.0), {0.0, 0.0, 0.0}, 40.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.max_cross_track, 0.02);
}

TEST(MpcClosedLoop, StraightPathWithInitialOffset) {
  const auto r = run(straight_map(0.0, 4.0), {0.0, -0.3, 0.2}, 40.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LE(r.max_cross_track, 0.31);
}

TEST(MpcClosedLoop, QuarterCircle) {
  const auto r = run(quarter_circle(2.0), {0.0, 0.0, 0.0}, 40.0);
  EXPECT_TRUE(r.reached);  // includes the goal yaw pi/2 within 0.2 rad
  EXPECT_LT(r.max_cross_track, 0.1);
}

TEST(MpcClosedLoop, RightAngleCorner) {
  const auto r = run(l_shape(), {0.0, 0.0, 0.0}, 60.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.max_cross_track, 0.35);
}

TEST(MpcClosedLoop, PassesObstacleNextToPath) {
  // A box corner 0.3 m beside the path: pure pursuit would brush it, the MPC keeps the clearance.
  std::vector<Point2D> box;
  for (double x = 1.8; x <= 2.4 + 1e-9; x += 0.05) box.push_back({x, 0.3});
  const auto r = run(straight_map(0.0, 4.0), {0.0, 0.0, 0.0}, 60.0, box);
  EXPECT_TRUE(r.reached);
  EXPECT_FALSE(r.collided);
  EXPECT_GT(r.min_clearance, 0.35);  // outside the safety monitor envelope
}

}  // namespace
