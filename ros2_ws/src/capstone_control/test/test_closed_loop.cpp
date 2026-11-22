// Closed-loop checks without a simulator: an ideal unicycle model driven by the
// pure pursuit controller through the same limiter and goal/progress checkers the
// ROS node uses. Catches regressions in the whole tracking chain in milliseconds.

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "capstone_control/checkers.hpp"
#include "capstone_control/command_limiter.hpp"
#include "capstone_control/pure_pursuit_controller.hpp"

using namespace capstone_control;

namespace {

struct RunResult {
  bool reached{false};
  bool stuck{false};
  double time{0.0};
  double max_cross_track{0.0};
  Pose2D final_pose;
};

/// Runs the control loop at 20 Hz. `speed_scale` multiplies the executed velocity to
/// imitate a model mismatch (e.g. 0.8 = the robot is 20% slower than commanded).
RunResult run(const Path &path, Pose2D pose, double timeout, double speed_scale = 1.0) {
  const double dt = 0.05;
  PurePursuitController controller;
  CommandLimiter limiter;
  GoalChecker goal_checker;
  ProgressChecker progress;
  PathTracker tracker(path, 1.5);
  Command last{};
  Command measured{};
  RunResult r;
  progress.reset(pose, 0.0);
  for (double t = 0.0; t < timeout; t += dt) {
    tracker.update(pose);
    r.max_cross_track = std::max(r.max_cross_track, std::abs(tracker.cross_track_error()));
    if (goal_checker.reached(pose, tracker.goal(), measured)) {
      r.reached = true;
      r.time = t;
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
    const auto desired = controller.compute(in);
    last = limiter.limit(desired.value_or(Command{}), last, dt);
    measured = Command{last.v * speed_scale, last.w * speed_scale};
    pose.x += measured.v * std::cos(pose.yaw) * dt;
    pose.y += measured.v * std::sin(pose.yaw) * dt;
    pose.yaw = normalize_angle(pose.yaw + measured.w * dt);
  }
  r.final_pose = pose;
  return r;
}

Path straight(double length) {
  Path p;
  for (double x = 0.0; x <= length + 1e-9; x += 0.05) p.push_back({x, 0.0, 0.0});
  return p;
}

Path quarter_circle(double radius) {
  // Left turn starting at the origin heading +x, centre at (0, radius).
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

}  // namespace

TEST(ClosedLoop, StraightPath) {
  const RunResult r = run(straight(5.0), {0.0, 0.0, 0.0}, 60.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.max_cross_track, 0.01);
  EXPECT_LT(r.time, 20.0);
}

TEST(ClosedLoop, StraightPathWithInitialOffset) {
  const RunResult r = run(straight(5.0), {0.0, -0.3, 0.2}, 60.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LE(r.max_cross_track, 0.31);  // never worse than the start
}

TEST(ClosedLoop, QuarterCircle) {
  const RunResult r = run(quarter_circle(2.0), {0.0, 0.0, 0.0}, 60.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.max_cross_track, 0.1);
}

TEST(ClosedLoop, RightAngleCorner) {
  const RunResult r = run(l_shape(), {0.0, 0.0, 0.0}, 60.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.max_cross_track, 0.35);  // pure pursuit cuts corners by design
}

TEST(ClosedLoop, StartsFacingAwayFromPath) {
  const RunResult r = run(straight(3.0), {0.0, 0.0, kPi}, 60.0);
  EXPECT_TRUE(r.reached);
}

TEST(ClosedLoop, AlignsToGoalHeadingDifferentFromPathDirection) {
  Path p = straight(2.0);
  p.back().yaw = -kPi / 2;
  const RunResult r = run(p, {0.0, 0.0, 0.0}, 60.0);
  EXPECT_TRUE(r.reached);
  EXPECT_LT(std::abs(normalize_angle(r.final_pose.yaw + kPi / 2)), 0.2);
}

TEST(ClosedLoop, SinglePosePathJustTurns) {
  const RunResult r = run({{0.05, 0.0, 1.0}}, {0.0, 0.0, 0.0}, 30.0);
  EXPECT_TRUE(r.reached);
}

TEST(ClosedLoop, ToleratesSpeedModelMismatch) {
  for (const double scale : {0.8, 1.2}) {
    const RunResult r = run(quarter_circle(2.0), {0.0, 0.0, 0.0}, 60.0, scale);
    EXPECT_TRUE(r.reached) << "speed scale " << scale;
  }
}

TEST(ClosedLoop, RobotThatCannotMoveIsReportedStuck) {
  const RunResult r = run(straight(3.0), {0.0, 0.0, 0.0}, 30.0, 0.0);
  EXPECT_TRUE(r.stuck);
  EXPECT_FALSE(r.reached);
}
