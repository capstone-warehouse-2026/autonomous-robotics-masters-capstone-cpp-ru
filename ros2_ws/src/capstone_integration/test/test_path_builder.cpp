#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_integration/path_builder.hpp"

namespace {

using capstone_integration::ControlCommand;
using capstone_integration::FollowerParams;
using capstone_integration::follow_path;
using capstone_integration::path_length;
using capstone_integration::Pose2D;
using capstone_integration::remaining_distance;
using capstone_integration::straight_path;

TEST(StraightPath, SamplesStartToGoal) {
  const auto path = straight_path(Pose2D{0.0, 0.0, 0.0}, Pose2D{2.0, 0.0, 0.0}, 0.5);
  ASSERT_EQ(path.size(), 5U);
  EXPECT_NEAR(path.front().x, 0.0, 1e-12);
  EXPECT_NEAR(path.back().x, 2.0, 1e-12);
  EXPECT_NEAR(path_length(path), 2.0, 1e-12);
  for (const auto &point : path) {
    EXPECT_NEAR(point.yaw, 0.0, 1e-12);
  }
}

TEST(StraightPath, KeepsGoalYawOnTheLastPose) {
  const auto path = straight_path(Pose2D{0.0, 0.0, 0.0}, Pose2D{0.0, 2.0, 1.5707963267948966}, 1.0);
  ASSERT_EQ(path.size(), 3U);
  EXPECT_NEAR(path[1].yaw, 1.5707963267948966, 1e-12);
  EXPECT_NEAR(path.back().yaw, 1.5707963267948966, 1e-12);
}

TEST(StraightPath, DegenerateGoalIsASinglePose) {
  const auto path = straight_path(Pose2D{1.0, 1.0, 0.0}, Pose2D{1.0, 1.0, 0.5}, 0.1);
  ASSERT_EQ(path.size(), 1U);
  EXPECT_NEAR(path.front().yaw, 0.5, 1e-12);
}

TEST(StraightPath, RejectsInvalidStep) {
  EXPECT_THROW(straight_path(Pose2D{}, Pose2D{1.0, 0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW(straight_path(Pose2D{}, Pose2D{1.0, 0.0, 0.0}, std::nan("")),
               std::invalid_argument);
}

TEST(RemainingDistance, CountsToTheEnd) {
  const auto path = straight_path(Pose2D{0.0, 0.0, 0.0}, Pose2D{5.0, 0.0, 0.0}, 0.1);
  EXPECT_NEAR(remaining_distance(path, Pose2D{0.0, 0.0, 0.0}), 5.0, 0.11);
  EXPECT_NEAR(remaining_distance(path, Pose2D{2.0, 0.0, 0.0}), 3.0, 0.11);
  EXPECT_NEAR(remaining_distance(path, Pose2D{5.0, 0.0, 0.0}), 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(remaining_distance({}, Pose2D{}), 0.0);
}

TEST(FollowPath, DrivesTowardsTheLookAheadPoint) {
  const auto path = straight_path(Pose2D{0.0, 0.0, 0.0}, Pose2D{5.0, 0.0, 0.0}, 0.1);
  const ControlCommand command = follow_path(path, Pose2D{0.0, 0.0, 0.0}, FollowerParams{});
  EXPECT_GT(command.v, 0.0);
  EXPECT_NEAR(command.w, 0.0, 1e-9);
}

TEST(FollowPath, TurnsInPlaceWhenFacingAway) {
  const auto path = straight_path(Pose2D{0.0, 0.0, 0.0}, Pose2D{5.0, 0.0, 0.0}, 0.1);
  const ControlCommand command = follow_path(path, Pose2D{0.0, 0.0, 3.141592653589793}, FollowerParams{});
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_GT(std::abs(command.w), 0.0);
}

TEST(FollowPath, StopsInsideTheArrivalRadius) {
  const auto path = straight_path(Pose2D{0.0, 0.0, 0.0}, Pose2D{5.0, 0.0, 0.0}, 0.1);
  const ControlCommand command = follow_path(path, Pose2D{5.05, 0.0, 0.0}, FollowerParams{});
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_DOUBLE_EQ(command.w, 0.0);
}

TEST(FollowPath, EmptyPathIsAStop) {
  const ControlCommand command = follow_path({}, Pose2D{}, FollowerParams{});
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_DOUBLE_EQ(command.w, 0.0);
}

TEST(FollowPath, RejectsInvalidParameters) {
  FollowerParams params;
  params.max_v = 0.0;
  EXPECT_THROW(follow_path({Pose2D{1.0, 0.0, 0.0}}, Pose2D{}, params), std::invalid_argument);
}

}  // namespace
