#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "capstone_control/path_tracker.hpp"

using capstone_control::Path;
using capstone_control::PathTracker;
using capstone_control::Pose2D;
using capstone_control::validate_path;

namespace {

Path straight_x(double length, double step) {
  Path p;
  for (double x = 0.0; x <= length + 1e-9; x += step) {
    p.push_back({x, 0.0, 0.0});
  }
  return p;
}

}  // namespace

TEST(ValidatePath, RejectsEmptyPath) { EXPECT_TRUE(validate_path({}, 1.0).has_value()); }

TEST(ValidatePath, AcceptsSinglePose) { EXPECT_FALSE(validate_path({{1.0, 2.0, 0.5}}, 1.0).has_value()); }

TEST(ValidatePath, RejectsNonFinitePose) {
  Path p = straight_x(2.0, 0.5);
  p[2].y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(validate_path(p, 1.0).has_value());
  p[2].y = 0.0;
  p[3].yaw = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(validate_path(p, 1.0).has_value());
}

TEST(ValidatePath, RejectsLargeGap) {
  const Path p{{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  EXPECT_TRUE(validate_path(p, 1.0).has_value());
  EXPECT_FALSE(validate_path(p, 2.0).has_value());
}

TEST(PathTracker, LengthAndProjectionOnStraightPath) {
  PathTracker t(straight_x(4.0, 0.5), 1.5);
  EXPECT_NEAR(t.total_length(), 4.0, 1e-9);
  t.update({1.2, 0.3, 0.0});
  EXPECT_NEAR(t.progress(), 1.2, 1e-9);
  EXPECT_NEAR(t.remaining_length(), 2.8, 1e-9);
  EXPECT_NEAR(t.cross_track_error(), 0.3, 1e-9);  // left of the path is positive
  t.update({1.4, -0.2, 0.0});
  EXPECT_NEAR(t.cross_track_error(), -0.2, 1e-9);
}

TEST(PathTracker, PointAheadInterpolatesAndClampsAtGoal) {
  PathTracker t(straight_x(2.0, 1.0), 1.5);
  t.update({0.2, 0.0, 0.0});
  const Pose2D p = t.point_ahead(0.5);
  EXPECT_NEAR(p.x, 0.7, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  const Pose2D end = t.point_ahead(10.0);
  EXPECT_NEAR(end.x, 2.0, 1e-9);
}

TEST(PathTracker, ProjectionDoesNotJumpBackOnLoopingPath) {
  // Out along y = 0, a U-turn, and back along y = 0.4 close to the start.
  Path p;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.25) p.push_back({x, 0.0, 0.0});
  for (double x = 3.0; x >= 0.0 - 1e-9; x -= 0.25) p.push_back({x, 0.4, 3.14});
  PathTracker t(p, 1.0);
  // Drive along the outbound leg; the return leg is 0.4 m away but beyond the window.
  for (double x = 0.0; x <= 2.0 + 1e-9; x += 0.1) {
    t.update({x, 0.25, 0.0});  // slightly closer to the return leg than to the outbound one
  }
  EXPECT_NEAR(t.progress(), 2.0, 1e-6);
}

TEST(PathTracker, SinglePosePath) {
  PathTracker t({{1.0, 1.0, 0.5}}, 1.5);
  t.update({0.0, 1.0, 0.0});
  EXPECT_DOUBLE_EQ(t.total_length(), 0.0);
  EXPECT_DOUBLE_EQ(t.remaining_length(), 0.0);
  EXPECT_NEAR(t.point_ahead(0.6).x, 1.0, 1e-12);
}
