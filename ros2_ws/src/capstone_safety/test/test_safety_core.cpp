// Unit tests of SafetyCore: the decision logic of the safety monitor without ROS, in milliseconds.

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include "capstone_safety/safety_core.hpp"

namespace {

using capstone_safety::Cmd;
using capstone_safety::Decision;
using capstone_safety::Odom;
using capstone_safety::Params;
using capstone_safety::Point;
using capstone_safety::Reason;
using capstone_safety::SafetyCore;
using capstone_safety::Scan;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Start times; the sim and steady clocks run independently.
constexpr double kT0 = 10.0;   // sim
constexpr double kS0 = 100.0;  // steady

Cmd make_cmd(double v, double w, double t_sim = kT0, double t_steady = kS0) {
  Cmd c;
  c.v = v;
  c.w = w;
  c.t_sim = t_sim;
  c.t_steady = t_steady;
  return c;
}

Odom make_odom(double v, double t_sim = kT0, double t_steady = kS0) {
  Odom o;
  o.v = v;
  o.t_sim = t_sim;
  o.t_steady = t_steady;
  return o;
}

Scan make_scan(std::vector<Point> pts, double t_sim = kT0, double t_steady = kS0) {
  Scan s;
  s.points = std::move(pts);
  s.t_sim = t_sim;
  s.t_steady = t_steady;
  return s;
}

// Wall across the heading at distance x (a point every 5 cm from y = -1 to y = 1).
std::vector<Point> wall_ahead(double x) {
  std::vector<Point> pts;
  for (int i = -20; i <= 20; ++i) pts.push_back({x, 0.05 * i});
  return pts;
}

// Wall along the heading on the left at distance y (from x = -1 to x = 2).
std::vector<Point> wall_left(double y) {
  std::vector<Point> pts;
  for (int i = -20; i <= 40; ++i) pts.push_back({0.05 * i, y});
  return pts;
}

// Core fed with fresh data and an empty scan.
class FreshCore : public ::testing::Test {
 protected:
  void feed(const Cmd &c, double odom_v = 0.0, std::vector<Point> pts = {}) {
    core.on_odom(make_odom(odom_v));
    core.on_scan(make_scan(std::move(pts)));
    core.on_cmd(c);
  }
  Decision step(double dt_sim = 0.01, double dt_steady = 0.01) {
    return core.step(kT0 + dt_sim, kS0 + dt_steady);
  }
  SafetyCore core;
};

void expect_halt(const Decision &d, Reason r) {
  EXPECT_EQ(d.reason, r) << "got " << capstone_safety::to_string(d.reason);
  EXPECT_DOUBLE_EQ(d.v, 0.0);
  EXPECT_DOUBLE_EQ(d.w, 0.0);
  EXPECT_TRUE(d.intervened());
}

// ---------------------------------------------------------------- parameters

TEST(SafetyParams, RejectsNonPositiveBrake) {
  Params p;
  p.a_brake = 0.0;
  EXPECT_THROW(SafetyCore{p}, std::invalid_argument);
}

TEST(SafetyParams, RejectsNonFiniteParam) {
  Params p;
  p.margin = kNan;
  EXPECT_THROW(SafetyCore{p}, std::invalid_argument);
}

TEST(SafetyParams, ReasonNamesMatchEnum) {
  EXPECT_EQ(capstone_safety::to_string(Reason::OK), "OK");
  EXPECT_EQ(capstone_safety::to_string(Reason::STALE_CMD), "STALE_CMD");
  EXPECT_EQ(capstone_safety::to_string(Reason::COLLISION_RISK), "COLLISION_RISK");
}

// ------------------------------------------------------------ stopping distance

TEST(StoppingDistance, MatchesFormula) {
  Params p;  // t_latency 0.15, a_brake 0.5
  EXPECT_NEAR(SafetyCore::stopping_distance(0.6, p), 0.6 * 0.15 + 0.36 / 1.0, 1e-12);
  EXPECT_NEAR(SafetyCore::stopping_distance(0.2, p), 0.2 * 0.15 + 0.04 / 1.0, 1e-12);
}

TEST(StoppingDistance, ZeroForNonPositiveSpeed) {
  Params p;
  EXPECT_DOUBLE_EQ(SafetyCore::stopping_distance(0.0, p), 0.0);
  EXPECT_DOUBLE_EQ(SafetyCore::stopping_distance(-0.3, p), 0.0);
}

// ------------------------------------------------------------- geometry

TEST(CollisionRisk, EmptyScanIsSafe) {
  EXPECT_FALSE(SafetyCore::collision_risk(0.6, 0.0, {}, Params{}));
}

TEST(CollisionRisk, WallAheadDependsOnSpeed) {
  Params p;
  // v = 0.5: d_stop = 0.075 + 0.25 = 0.325; wall left at 0.6 - 0.325 = 0.275 < 0.35 -> risk.
  EXPECT_TRUE(SafetyCore::collision_risk(0.5, 0.0, wall_ahead(0.6), p));
  // v = 0.05: d_stop ~ 0.01; wall left at ~0.59 > 0.35 -> allowed.
  EXPECT_FALSE(SafetyCore::collision_risk(0.05, 0.0, wall_ahead(0.6), p));
}

TEST(CollisionRisk, FarWallIsSafeAtFullSpeed) {
  // v = 0.6: d_stop = 0.09 + 0.36 = 0.45; wall left at 1.0 - 0.45 = 0.55 > 0.35.
  EXPECT_FALSE(SafetyCore::collision_risk(0.6, 0.0, wall_ahead(1.0), Params{}));
}

TEST(CollisionRisk, ObstacleBehindDoesNotBlockForward) {
  // The point behind is inside the clearance, but the robot drives away from it.
  EXPECT_FALSE(SafetyCore::collision_risk(0.5, 0.0, {{-0.30, 0.0}}, Params{}));
}

TEST(CollisionRisk, SideWallBlocksTurnTowardsIt) {
  Params p;
  const auto wall = wall_left(0.45);
  // Straight along the wall: it stays at least 0.45 > 0.35 away -> allowed.
  EXPECT_FALSE(SafetyCore::collision_risk(0.5, 0.0, wall, p));
  // Sharp left turn (k = 1.2 / 0.5 = 2.4 1/m) leads towards the wall -> risk.
  EXPECT_TRUE(SafetyCore::collision_risk(0.5, 1.2, wall, p));
  // Right turn, away from the wall -> allowed.
  EXPECT_FALSE(SafetyCore::collision_risk(0.5, -1.2, wall, p));
}

TEST(CollisionRisk, RotationInPlaceNearWallAllowed) {
  // Wall at 0.30 (between R = 0.25 and C = 0.35), v = 0: turning in place is allowed.
  EXPECT_FALSE(SafetyCore::collision_risk(0.0, 1.0, wall_ahead(0.30), Params{}));
}

TEST(CollisionRisk, ContactIsAlwaysRisk) {
  // A point inside the robot disc is a contact, even when standing.
  EXPECT_TRUE(SafetyCore::collision_risk(0.0, 1.0, {{0.20, 0.0}}, Params{}));
}

TEST(CollisionRisk, IgnoresNonFinitePoints) {
  EXPECT_FALSE(SafetyCore::collision_risk(0.6, 0.0, {{kNan, 0.0}, {0.3, kInf}}, Params{}));
}

// ------------------------------------------------------------ readiness and freshness

TEST_F(FreshCore, NotReadyWithoutInputs) {
  expect_halt(step(), Reason::NOT_READY);
}

TEST_F(FreshCore, NotReadyWithoutScan) {
  core.on_odom(make_odom(0.0));
  core.on_cmd(make_cmd(0.3, 0.0));
  expect_halt(step(), Reason::NOT_READY);
}

TEST_F(FreshCore, PassesValidCommand) {
  feed(make_cmd(0.3, 0.2));
  const Decision d = step();
  EXPECT_EQ(d.reason, Reason::OK);
  EXPECT_DOUBLE_EQ(d.v, 0.3);
  EXPECT_DOUBLE_EQ(d.w, 0.2);
  EXPECT_FALSE(d.intervened());
}

TEST_F(FreshCore, StaleScan) {
  feed(make_cmd(0.3, 0.0));
  core.on_cmd(make_cmd(0.3, 0.0, kT0 + 0.30, kS0 + 0.30));  // the command is fresh
  expect_halt(core.step(kT0 + 0.31, kS0 + 0.31), Reason::STALE_SCAN);
}

TEST_F(FreshCore, StaleOdom) {
  feed(make_cmd(0.3, 0.0));
  core.on_scan(make_scan({}, kT0 + 0.30, kS0 + 0.30));
  core.on_cmd(make_cmd(0.3, 0.0, kT0 + 0.30, kS0 + 0.30));
  expect_halt(core.step(kT0 + 0.31, kS0 + 0.31), Reason::STALE_ODOM);
}

TEST_F(FreshCore, StaleCommandBySimTime) {
  feed(make_cmd(0.3, 0.0));
  expect_halt(step(0.16, 0.16), Reason::STALE_CMD);
}

TEST_F(FreshCore, StaleCommandBySteadyTimeWhenSimFrozen) {
  // Simulation paused: sim time stands still while steady time advances by 350 ms.
  feed(make_cmd(0.3, 0.0));
  expect_halt(step(0.0, 0.35), Reason::STALE_CMD);
}

TEST_F(FreshCore, CommandAt140msStillFresh) {
  feed(make_cmd(0.3, 0.0));
  EXPECT_EQ(step(0.14, 0.14).reason, Reason::OK);
}

// ------------------------------------------------------------ latch

TEST_F(FreshCore, StaleCommandLatchesUntilNewStamp) {
  feed(make_cmd(0.3, 0.0));
  expect_halt(step(0.20, 0.20), Reason::STALE_CMD);
  EXPECT_TRUE(core.latched());

  // Refresh the sensors so that only the latch is tested.
  core.on_odom(make_odom(0.0, kT0 + 0.25, kS0 + 0.25));
  core.on_scan(make_scan({}, kT0 + 0.25, kS0 + 0.25));

  // The same command with the old stamp arrives again (e.g. delayed in a queue) -> still stopped.
  core.on_cmd(make_cmd(0.3, 0.0, kT0, kS0 + 0.25));
  expect_halt(core.step(kT0 + 0.26, kS0 + 0.26), Reason::STALE_CMD);

  // A new command with a fresh stamp releases the latch.
  core.on_cmd(make_cmd(0.2, 0.0, kT0 + 0.27, kS0 + 0.27));
  EXPECT_FALSE(core.latched());
  const Decision d = core.step(kT0 + 0.28, kS0 + 0.28);
  EXPECT_EQ(d.reason, Reason::OK);
  EXPECT_DOUBLE_EQ(d.v, 0.2);
}

TEST_F(FreshCore, HaltIsRepeatedEveryTick) {
  // The monitor does not publish one zero and fall silent: every tick yields a zero command.
  feed(make_cmd(0.3, 0.0));
  for (int i = 0; i < 10; ++i) {
    const double t = 0.20 + 0.05 * i;
    // Sensors keep arriving, only the controller is silent.
    core.on_odom(make_odom(0.0, kT0 + t, kS0 + t));
    core.on_scan(make_scan({}, kT0 + t, kS0 + t));
    expect_halt(step(t, t), Reason::STALE_CMD);
  }
}

// ------------------------------------------------------------ invalid commands

TEST_F(FreshCore, NanCommand) {
  feed(make_cmd(kNan, 0.0));
  expect_halt(step(), Reason::NONFINITE);
}

TEST_F(FreshCore, InfAngularCommand) {
  feed(make_cmd(0.2, kInf));
  expect_halt(step(), Reason::NONFINITE);
}

TEST_F(FreshCore, NonfiniteDoesNotLatch) {
  feed(make_cmd(kNan, 0.0));
  expect_halt(step(), Reason::NONFINITE);
  core.on_cmd(make_cmd(0.2, 0.0, kT0 + 0.02, kS0 + 0.02));
  EXPECT_EQ(core.step(kT0 + 0.03, kS0 + 0.03).reason, Reason::OK);
}

TEST_F(FreshCore, OverSpeedIsLimit) {
  feed(make_cmd(0.8, 0.0));
  expect_halt(step(), Reason::LIMIT);
}

TEST_F(FreshCore, ReverseIsLimit) {
  feed(make_cmd(-0.1, 0.0));
  expect_halt(step(), Reason::LIMIT);
}

TEST_F(FreshCore, OverAngularIsLimit) {
  feed(make_cmd(0.0, -1.5));
  expect_halt(step(), Reason::LIMIT);
}

TEST_F(FreshCore, ExactLimitsPass) {
  feed(make_cmd(0.6, 1.2));
  EXPECT_EQ(step().reason, Reason::OK);
}

// ------------------------------------------------------------ collisions in step()

TEST_F(FreshCore, WallAheadHalts) {
  feed(make_cmd(0.5, 0.0), 0.5, wall_ahead(0.6));
  expect_halt(step(), Reason::COLLISION_RISK);
}

TEST_F(FreshCore, MeasuredSpeedUsedForEnvelope) {
  // Slow command, but odom still reports 0.5 m/s -> the stopping distance uses 0.5.
  feed(make_cmd(0.05, 0.0), 0.5, wall_ahead(0.6));
  expect_halt(step(), Reason::COLLISION_RISK);
}

TEST_F(FreshCore, SlowApproachAllowed) {
  feed(make_cmd(0.05, 0.0), 0.05, wall_ahead(0.6));
  EXPECT_EQ(step().reason, Reason::OK);
}

// ------------------------------------------------------------ check order and reset

TEST_F(FreshCore, StaleBeatsNonfinite) {
  // A command that is both stale and NaN is reported as STALE_CMD (checked first).
  feed(make_cmd(kNan, 0.0));
  expect_halt(step(0.20, 0.20), Reason::STALE_CMD);
}

TEST_F(FreshCore, TimeJumpBackwardsResets) {
  feed(make_cmd(0.3, 0.0));
  // Simulation restarted: sim time is near zero again.
  expect_halt(core.step(0.5, kS0 + 0.02), Reason::TIME_JUMP);
  // The state is cleared: old data are not used.
  expect_halt(core.step(0.55, kS0 + 0.07), Reason::NOT_READY);
}

TEST_F(FreshCore, LimitBeatsCollision) {
  // v = 0.8 > v_max = 0.6 -> LIMIT. Odom 0.5 m/s and a wall at 0.6 m also give COLLISION_RISK
  // (see WallAheadHalts); LIMIT is checked first.
  feed(make_cmd(0.8, 0.0), 0.5, wall_ahead(0.6));
  expect_halt(step(), Reason::LIMIT);
}

TEST_F(FreshCore, ResetClearsLatch) {
  // Latch: command at time 0, step after 200 ms (> 150 ms).
  feed(make_cmd(0.3, 0.0));
  expect_halt(step(0.20, 0.20), Reason::STALE_CMD);
  ASSERT_TRUE(core.latched());  // nothing to reset otherwise

  core.reset();

  // Latch released and no data left.
  EXPECT_FALSE(core.latched());
  expect_halt(step(0.21, 0.21), Reason::NOT_READY);
}

// Only a new command releases the latch, fresh sensors do not. Regression: odom/scan used to release
// it, so after a simulation pause a re-delivered old command was resumed.
TEST_F(FreshCore, SensorDataDoesNotReleaseLatch) {
  feed(make_cmd(0.3, 0.0));
  expect_halt(step(0.0, 0.35), Reason::STALE_CMD);  // sim paused, steady time ran on
  ASSERT_TRUE(core.latched());

  core.on_odom(make_odom(0.0, kT0 + 0.01, kS0 + 0.36));
  core.on_scan(make_scan({}, kT0 + 0.01, kS0 + 0.36));
  EXPECT_TRUE(core.latched());

  core.on_cmd(make_cmd(0.3, 0.0, kT0, kS0 + 0.36));  // the old command again
  expect_halt(step(0.02, 0.37), Reason::STALE_CMD);
}

}  // namespace
