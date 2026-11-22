#include <cmath>

#include <gtest/gtest.h>

#include "capstone_control/pure_pursuit_controller.hpp"

using capstone_control::ControlInput;
using capstone_control::Path;
using capstone_control::PathTracker;
using capstone_control::Pose2D;
using capstone_control::PurePursuitController;
using capstone_control::PurePursuitParams;

namespace {

Path straight_x(double length) {
  Path p;
  for (double x = 0.0; x <= length + 1e-9; x += 0.1) p.push_back({x, 0.0, 0.0});
  return p;
}

ControlInput input_for(PathTracker &t, const Pose2D &pose) {
  t.update(pose);
  ControlInput in;
  in.pose = pose;
  in.path = &t;
  return in;
}

}  // namespace

TEST(PurePursuit, DrivesStraightWhenOnPath) {
  PurePursuitController c;
  PathTracker t(straight_x(5.0), 1.5);
  const auto cmd = c.compute(input_for(t, {0.0, 0.0, 0.0}));
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->v, c.params().max_speed, 1e-9);
  EXPECT_NEAR(cmd->w, 0.0, 1e-9);
}

TEST(PurePursuit, SteersBackTowardsPath) {
  PurePursuitController c;
  PathTracker t(straight_x(5.0), 1.5);
  const auto right_of_path = c.compute(input_for(t, {1.0, -0.2, 0.0}));
  ASSERT_TRUE(right_of_path.has_value());
  EXPECT_GT(right_of_path->v, 0.0);
  EXPECT_GT(right_of_path->w, 0.0);  // turn left
  const auto left_of_path = c.compute(input_for(t, {1.0, 0.2, 0.0}));
  ASSERT_TRUE(left_of_path.has_value());
  EXPECT_LT(left_of_path->w, 0.0);  // turn right
}

TEST(PurePursuit, TurnsOnTheSpotWhenPathIsBehind) {
  PurePursuitController c;
  PathTracker t(straight_x(5.0), 1.5);
  const auto cmd = c.compute(input_for(t, {0.0, 0.0, 3.0}));  // facing almost backwards
  ASSERT_TRUE(cmd.has_value());
  EXPECT_DOUBLE_EQ(cmd->v, 0.0);
  EXPECT_NE(cmd->w, 0.0);
  EXPECT_LE(std::abs(cmd->w), c.params().max_yaw_rate);
}

TEST(PurePursuit, SlowsDownNearTheEnd) {
  PurePursuitController c;
  // Separate trackers: the projection search window (1.5 m) does not allow a 3.6 m jump.
  PathTracker t_far(straight_x(5.0), 1.5);
  PathTracker t_near(straight_x(5.0), 10.0);
  const auto far = c.compute(input_for(t_far, {1.0, 0.0, 0.0}));
  const auto near = c.compute(input_for(t_near, {4.6, 0.0, 0.0}));
  ASSERT_TRUE(far && near);
  EXPECT_LT(near->v, far->v);
  EXPECT_GE(near->v, c.params().min_speed);
}

TEST(PurePursuit, AlignsYawAtGoalAndLatches) {
  PurePursuitController c;
  Path p = straight_x(2.0);
  p.back().yaw = 1.5;  // goal heading differs from the path direction
  PathTracker t(p, 1.5);
  const auto turn = c.compute(input_for(t, {1.95, 0.0, 0.0}));
  ASSERT_TRUE(turn.has_value());
  EXPECT_TRUE(c.aligning());
  EXPECT_DOUBLE_EQ(turn->v, 0.0);
  EXPECT_GT(turn->w, 0.0);
  // Still aligning even if the robot drifted slightly outside the final tolerance.
  const auto still = c.compute(input_for(t, {1.8, 0.0, 0.5}));
  ASSERT_TRUE(still.has_value());
  EXPECT_DOUBLE_EQ(still->v, 0.0);
  // Aligned: zero command.
  const auto done = c.compute(input_for(t, {1.95, 0.0, 1.5}));
  ASSERT_TRUE(done.has_value());
  EXPECT_DOUBLE_EQ(done->v, 0.0);
  EXPECT_DOUBLE_EQ(done->w, 0.0);
  c.reset();
  EXPECT_FALSE(c.aligning());
}

TEST(PurePursuit, RespectsYawRateLimitOnSharpTurn) {
  PurePursuitParams params;
  params.max_yaw_rate = 0.5;
  PurePursuitController c(params);
  // Path turns sharply left right in front of the robot.
  Path p{{0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.2, 0.2, 1.57}, {0.2, 0.4, 1.57}, {0.2, 0.6, 1.57}, {0.2, 0.8, 1.57}};
  PathTracker t(p, 1.5);
  const auto cmd = c.compute(input_for(t, {0.0, 0.0, 0.0}));
  ASSERT_TRUE(cmd.has_value());
  EXPECT_LE(std::abs(cmd->w), 0.5 + 1e-9);
}

TEST(PurePursuit, NoValidCommandWithoutPathOrWithNonFinitePose) {
  PurePursuitController c;
  ControlInput in;
  EXPECT_FALSE(c.compute(in).has_value());
  PathTracker t(straight_x(1.0), 1.5);
  in.path = &t;
  in.pose.x = std::nan("");
  EXPECT_FALSE(c.compute(in).has_value());
}

TEST(PurePursuit, RejectsInvalidParameters) {
  PurePursuitParams params;
  params.lookahead = 0.0;
  EXPECT_THROW(PurePursuitController{params}, std::invalid_argument);
}
