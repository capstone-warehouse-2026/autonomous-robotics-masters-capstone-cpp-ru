#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "capstone_integration/kinematics.hpp"

namespace {

using capstone_integration::angle_diff;
using capstone_integration::clamp;
using capstone_integration::distance;
using capstone_integration::goal_reached;
using capstone_integration::integrate_twist;
using capstone_integration::kPi;
using capstone_integration::normalize_angle;
using capstone_integration::Pose2D;

TEST(NormalizeAngle, KeepsValuesInsidePi) {
  EXPECT_NEAR(normalize_angle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(normalize_angle(kPi + 0.1), -kPi + 0.1, 1e-12);
  EXPECT_NEAR(normalize_angle(-kPi - 0.1), kPi - 0.1, 1e-12);
  EXPECT_NEAR(normalize_angle(3.0 * kPi), -kPi, 1e-12);
}

TEST(NormalizeAngle, RejectsNonFinite) {
  EXPECT_THROW(normalize_angle(std::nan("")), std::invalid_argument);
}

TEST(AngleDiff, TakesTheShortWay) {
  EXPECT_NEAR(angle_diff(0.1, 0.2), -0.1, 1e-12);
  EXPECT_NEAR(angle_diff(-3.0, 3.0), 0.283185307179586, 1e-12);
}

TEST(IntegrateTwist, StraightLine) {
  const Pose2D start{0.0, 0.0, kPi / 2.0};
  const Pose2D pose = integrate_twist(start, 1.0, 0.0, 2.0);
  EXPECT_NEAR(pose.x, 0.0, 1e-12);
  EXPECT_NEAR(pose.y, 2.0, 1e-12);
  EXPECT_NEAR(pose.yaw, kPi / 2.0, 1e-12);
}

TEST(IntegrateTwist, QuarterCircle) {
  const Pose2D start{0.0, 0.0, 0.0};
  const Pose2D pose = integrate_twist(start, 1.0, kPi / 2.0, 1.0);
  const double radius = 2.0 / kPi;
  EXPECT_NEAR(pose.x, radius, 1e-12);
  EXPECT_NEAR(pose.y, radius, 1e-12);
  EXPECT_NEAR(pose.yaw, kPi / 2.0, 1e-12);
}

TEST(IntegrateTwist, RejectsInvalidStep) {
  const Pose2D start{1.0, 2.0, 0.3};
  EXPECT_EQ(integrate_twist(start, 1.0, 0.0, 0.0).x, 1.0);
  EXPECT_EQ(integrate_twist(start, std::nan(""), 0.0, 0.1).x, 1.0);
  EXPECT_EQ(integrate_twist(start, 1.0, std::numeric_limits<double>::infinity(), 0.1).x, 1.0);
}

TEST(GoalReached, PositionAndYaw) {
  const Pose2D goal{5.0, 5.0, kPi};
  EXPECT_TRUE(goal_reached(Pose2D{5.1, 5.0, kPi - 0.1}, goal, 0.25, 0.2));
  EXPECT_FALSE(goal_reached(Pose2D{5.4, 5.0, kPi}, goal, 0.25, 0.2));
  EXPECT_FALSE(goal_reached(Pose2D{5.0, 5.0, kPi - 0.5}, goal, 0.25, 0.2));
  EXPECT_FALSE(goal_reached(Pose2D{5.0, 5.0, kPi}, goal, 0.0, 0.2));
}

TEST(DistanceAndClamp, BasicBehaviour) {
  EXPECT_NEAR(distance(Pose2D{0.0, 0.0, 0.0}, Pose2D{3.0, 4.0, 0.0}), 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(clamp(2.0, -1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(clamp(-2.0, -1.0, 1.0), -1.0);
  EXPECT_THROW(clamp(0.0, 1.0, -1.0), std::invalid_argument);
}

}  // namespace
