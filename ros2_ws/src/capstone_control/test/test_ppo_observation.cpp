// Unit tests of the PPO network input in C++: the same cases and numbers as ml/control/test_spec.py.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include "capstone_control/ppo_observation.hpp"

namespace {

namespace ppo = capstone_control::ppo;
using capstone_control::Command;
using capstone_control::ControlInput;
using capstone_control::kPi;
using capstone_control::Path;
using capstone_control::PathTracker;
using capstone_control::Point2D;
using capstone_control::Pose2D;

constexpr double kTol = 1e-6;

// Points 0.2..2.0 m straight ahead at lateral offset y.
ppo::PathFeatures ahead(double y = 0.0, double remaining = 3.0, double goal_yaw = 0.0) {
  ppo::PathFeatures f;
  for (int i = 0; i < ppo::kPathPoints; ++i) f.points[i] = {0.2 * (i + 1), y};
  f.remaining = remaining;
  f.goal_yaw = goal_yaw;
  return f;
}

std::vector<float> obs_of(const ppo::PathFeatures &f, std::vector<Point2D> obstacles = {}, Command v = {}) {
  return ppo::build_observation(f, obstacles, v);
}

double sector(const std::vector<float> &obs, int i) { return obs[23 + i] * ppo::kRangeMax; }

TEST(PpoObservation, LayoutAndVelocity) {
  const auto obs = obs_of(ahead(), {}, Command{0.25, -0.5});
  ASSERT_EQ(obs.size(), 61u);
  EXPECT_EQ(ppo::kObsSize, 61);
  EXPECT_NEAR(obs[59], 0.5, kTol);
  EXPECT_NEAR(obs[60], -0.5, kTol);
}

TEST(PpoObservation, PathPointsAreScaled) {
  const auto obs = obs_of(ahead(0.3));
  ASSERT_EQ(obs.size(), 61u);
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(obs[2 * i] * ppo::kPathScale, 0.2 * (i + 1), kTol) << "i=" << i;
    EXPECT_NEAR(obs[2 * i + 1] * ppo::kPathScale, 0.3, kTol) << "i=" << i;
  }
}

TEST(PpoObservation, RemainingIsScaledAndCapped) {
  EXPECT_NEAR(obs_of(ahead(0.0, 0.5))[20], 0.1, kTol);
  EXPECT_NEAR(obs_of(ahead(0.0, 8.0))[20], 1.0, kTol);
}

TEST(PpoObservation, GoalYawAsCosSin) {
  auto obs = obs_of(ahead(0.0, 3.0, kPi / 2));
  ASSERT_EQ(obs.size(), 61u);
  EXPECT_NEAR(obs[21], 0.0, kTol);
  EXPECT_NEAR(obs[22], 1.0, kTol);
  obs = obs_of(ahead(0.0, 3.0, -kPi));
  EXPECT_NEAR(obs[21], -1.0, kTol);
  EXPECT_NEAR(obs[22], 0.0, kTol);
}

TEST(PpoObservation, SectorsEmptyAndBearing) {
  // Bearings away from sector borders, so that rounding cannot move a point to the next sector.
  const auto obs = obs_of(ahead(), {{1.5, 0.1}, {2.0, 0.15}, {-0.1, 1.2}, {-0.9, -0.001}, {5.0, 5.0}});
  ASSERT_EQ(obs.size(), 61u);
  EXPECT_NEAR(sector(obs, 18), std::hypot(1.5, 0.1), kTol);  // bearing ~4 deg, the nearer point
  EXPECT_NEAR(sector(obs, 27), std::hypot(0.1, 1.2), kTol);  // bearing ~95 deg
  EXPECT_NEAR(sector(obs, 0), std::hypot(0.9, 0.001), kTol);  // bearing just above -180 deg
  EXPECT_NEAR(sector(obs, 31), ppo::kRangeMax, kTol);         // (5, 5) is beyond kRangeMax
  int occupied = 0;
  for (int i = 0; i < ppo::kSectors; ++i) occupied += sector(obs, i) < ppo::kRangeMax - kTol;
  EXPECT_EQ(occupied, 3);
}

TEST(PpoObservation, BearingPiGoesToLastSector) {
  EXPECT_NEAR(sector(obs_of(ahead(), {{-1.0, 0.0}}), 35), 1.0, kTol);
}

TEST(PpoAction, ToTarget) {
  auto c = ppo::action_to_target(1.0f, 0.0f);
  EXPECT_NEAR(c.v, 0.5, kTol);
  EXPECT_NEAR(c.w, 0.0, kTol);
  c = ppo::action_to_target(-1.0f, -1.0f);
  EXPECT_NEAR(c.v, 0.0, kTol);
  EXPECT_NEAR(c.w, -1.0, kTol);
  c = ppo::action_to_target(0.0f, 0.5f);
  EXPECT_NEAR(c.v, 0.25, kTol);
  EXPECT_NEAR(c.w, 0.5, kTol);
  c = ppo::action_to_target(5.0f, -5.0f);  // clipped
  EXPECT_NEAR(c.v, 0.5, kTol);
  EXPECT_NEAR(c.w, -1.0, kTol);
}

TEST(PpoPathFeatures, FromTracker) {
  // Path along world +x at y = 1; the robot stands on it at x = 1, facing +y: the path runs to its right.
  Path path;
  for (int i = 0; i <= 50; ++i) path.push_back({0.1 * i, 1.0, 0.0});
  PathTracker tracker(path, 1.5);
  const Pose2D robot{1.0, 1.0, kPi / 2};
  tracker.update(robot);
  ControlInput in;
  in.pose = robot;
  in.path = &tracker;
  const auto f = ppo::path_features(in);
  EXPECT_NEAR(f.points[0].x, 0.0, 1e-9);
  EXPECT_NEAR(f.points[0].y, -0.2, 1e-9);
  EXPECT_NEAR(f.points[9].x, 0.0, 1e-9);
  EXPECT_NEAR(f.points[9].y, -2.0, 1e-9);
  EXPECT_NEAR(f.remaining, 4.0, 1e-9);
  EXPECT_NEAR(f.goal_yaw, -kPi / 2, 1e-9);
  // The ControlInput overload combines both steps.
  in.velocity = Command{0.25, 0.0};
  const auto obs = ppo::build_observation(in);
  ASSERT_EQ(obs.size(), 61u);
  EXPECT_NEAR(obs[1] * ppo::kPathScale, -0.2, kTol);
  EXPECT_NEAR(obs[59], 0.5, kTol);
}

}  // namespace
