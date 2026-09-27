#include <gtest/gtest.h>

#include "capstone_sim/diff_drive_kinematics.hpp"

namespace {

using capstone_sim::diff_drive_wheel_speeds;
using capstone_sim::limit_wheel_speeds;

// TIAGo Base geometry (simulation/worlds/warehouse.wbt, resource/tiago_base.urdf).
constexpr double kRadius = 0.0985;
constexpr double kSeparation = 0.4044;

TEST(DiffDriveKinematics, ForwardDrivesBothWheelsEqually) {
  const auto speeds = diff_drive_wheel_speeds(0.5, 0.0, kRadius, kSeparation);
  EXPECT_DOUBLE_EQ(speeds.left, 0.5 / kRadius);
  EXPECT_DOUBLE_EQ(speeds.right, 0.5 / kRadius);
}

TEST(DiffDriveKinematics, CounterclockwiseTurnSpinsRightWheelForward) {
  const auto speeds = diff_drive_wheel_speeds(0.0, 1.0, kRadius, kSeparation);
  EXPECT_DOUBLE_EQ(speeds.right, kSeparation / 2.0 / kRadius);
  EXPECT_DOUBLE_EQ(speeds.left, -speeds.right);
}

TEST(DiffDriveKinematics, StopIsZero) {
  const auto speeds = diff_drive_wheel_speeds(0.0, 0.0, kRadius, kSeparation);
  EXPECT_DOUBLE_EQ(speeds.left, 0.0);
  EXPECT_DOUBLE_EQ(speeds.right, 0.0);
}

TEST(DiffDriveKinematics, LimitKeepsCurvatureAndCapsFasterWheel) {
  const auto limited = limit_wheel_speeds({4.0, 20.0}, 10.0);
  EXPECT_DOUBLE_EQ(limited.right, 10.0);
  EXPECT_DOUBLE_EQ(limited.left, 2.0);
}

TEST(DiffDriveKinematics, LimitLeavesFeasibleSpeedsUnchanged) {
  const auto limited = limit_wheel_speeds({-3.0, 5.0}, 10.0);
  EXPECT_DOUBLE_EQ(limited.left, -3.0);
  EXPECT_DOUBLE_EQ(limited.right, 5.0);
}

}  // namespace
