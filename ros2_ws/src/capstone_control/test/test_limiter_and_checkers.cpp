#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "capstone_control/checkers.hpp"
#include "capstone_control/command_limiter.hpp"

using capstone_control::Command;
using capstone_control::CommandLimiter;
using capstone_control::CommandLimits;
using capstone_control::GoalChecker;
using capstone_control::Pose2D;
using capstone_control::ProgressChecker;

TEST(CommandLimiter, ClampsSpeedAndForbidsReverseByDefault) {
  CommandLimiter l;
  const Command fast = l.limit({5.0, 5.0}, {0.5, 1.0}, 1.0);
  EXPECT_DOUBLE_EQ(fast.v, 0.5);
  EXPECT_DOUBLE_EQ(fast.w, 1.0);
  const Command back = l.limit({-0.3, 0.0}, {0.0, 0.0}, 1.0);
  EXPECT_DOUBLE_EQ(back.v, 0.0);
}

TEST(CommandLimiter, LimitsAccelerationAndDeceleration) {
  CommandLimits limits;
  limits.max_accel = 0.5;
  limits.max_decel = 1.0;
  limits.max_yaw_accel = 2.0;
  CommandLimiter l(limits);
  const Command up = l.limit({0.5, 1.0}, {0.0, 0.0}, 0.05);
  EXPECT_NEAR(up.v, 0.025, 1e-12);  // 0.5 m/s^2 * 0.05 s
  EXPECT_NEAR(up.w, 0.1, 1e-12);    // 2 rad/s^2 * 0.05 s
  const Command down = l.limit({0.0, 0.0}, {0.5, 0.0}, 0.05);
  EXPECT_NEAR(down.v, 0.45, 1e-12);  // 1 m/s^2 * 0.05 s
}

TEST(CommandLimiter, NonFiniteDesiredGivesZero) {
  CommandLimiter l;
  const Command c = l.limit({std::numeric_limits<double>::quiet_NaN(), 0.3}, {0.4, 0.0}, 0.05);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
  EXPECT_DOUBLE_EQ(c.w, 0.0);
}

TEST(GoalChecker, RequiresPositionYawAndStop) {
  GoalChecker g;
  const Pose2D goal{1.0, 1.0, 0.0};
  EXPECT_TRUE(g.reached({1.1, 1.1, 0.1}, goal, {0.0, 0.0}));
  EXPECT_FALSE(g.reached({1.3, 1.0, 0.0}, goal, {0.0, 0.0}));  // 0.3 m away
  EXPECT_FALSE(g.reached({1.0, 1.0, 0.3}, goal, {0.0, 0.0}));  // yaw off by 0.3 rad
  EXPECT_FALSE(g.reached({1.0, 1.0, 0.0}, goal, {0.2, 0.0}));  // still moving
  EXPECT_FALSE(g.reached({1.0, 1.0, 0.0}, goal, {0.0, 0.5}));  // still turning
  // Yaw wraps around: 3.1 and -3.1 are 0.08 rad apart.
  EXPECT_TRUE(g.reached({0.0, 0.0, 3.1}, {0.0, 0.0, -3.1}, {0.0, 0.0}));
}

TEST(ProgressChecker, DetectsStuckRobotAfterTimeAllowance) {
  ProgressChecker p;  // 0.1 m or 0.5 rad within 10 s
  const Pose2D start{0.0, 0.0, 0.0};
  p.reset(start, 0.0);
  EXPECT_TRUE(p.check({0.05, 0.0, 0.0}, 9.0));
  EXPECT_FALSE(p.check({0.05, 0.0, 0.0}, 10.5));
}

TEST(ProgressChecker, MovementOrRotationRestartsTheWindow) {
  ProgressChecker p;
  p.reset({0.0, 0.0, 0.0}, 0.0);
  EXPECT_TRUE(p.check({0.2, 0.0, 0.0}, 8.0));   // moved: new window from t = 8
  EXPECT_TRUE(p.check({0.2, 0.0, 0.0}, 17.0));
  EXPECT_TRUE(p.check({0.2, 0.0, 0.6}, 17.5));  // turned on the spot: new window
  EXPECT_TRUE(p.check({0.2, 0.0, 0.6}, 27.0));
  EXPECT_FALSE(p.check({0.2, 0.0, 0.6}, 28.0));
}

TEST(ProgressChecker, SimulationTimeResetStartsNewWindow) {
  ProgressChecker p;
  p.reset({0.0, 0.0, 0.0}, 100.0);
  EXPECT_TRUE(p.check({0.0, 0.0, 0.0}, 1.0));  // time went backwards
  EXPECT_TRUE(p.check({0.0, 0.0, 0.0}, 10.0));
  EXPECT_FALSE(p.check({0.0, 0.0, 0.0}, 12.0));
}
