#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "capstone_webots_bridge/odometry.hpp"
using capstone_webots_bridge::WheelOdometry;
using capstone_webots_bridge::command_is_fresh;

TEST(WheelOdometry, MeasuredEncoderMotionProducesStraightDistance) {
  WheelOdometry odom(0.1, 0.4);
  ASSERT_TRUE(odom.update(5.0, 10.0, 0.02));
  ASSERT_TRUE(odom.update(6.0, 11.0, 0.1));
  EXPECT_NEAR(odom.x, 0.1, 1e-12);
  EXPECT_NEAR(odom.y, 0, 1e-12);
  EXPECT_NEAR(odom.velocity, 1, 1e-12);
}
TEST(WheelOdometry, ExactArcAndInPlaceRotation) {
  WheelOdometry odom(0.1, 0.4);
  odom.update(0, 0, 0.02);
  odom.update(0, 2, 0.1);
  EXPECT_NEAR(odom.yaw, 0.5, 1e-12);
  EXPECT_NEAR(odom.x, 0.2 * std::sin(0.5), 1e-12);
  EXPECT_NEAR(odom.y, 0.2 * (1-std::cos(0.5)), 1e-12);
  const double x = odom.x, y = odom.y;
  odom.update(-1, 3, 0.1);
  EXPECT_NEAR(odom.x, x, 1e-12);
  EXPECT_NEAR(odom.y, y, 1e-12);
}
TEST(WheelOdometry, ResetUsesNewEncoderBaselineWithoutJump) {
  WheelOdometry odom(0.1, 0.4);
  odom.update(0, 0, 0.02);
  odom.update(10, 10, 0.1);
  odom.reset();
  odom.update(70, 90, 0.02);
  EXPECT_DOUBLE_EQ(odom.x, 0);
  odom.update(71, 91, 0.1);
  EXPECT_NEAR(odom.x, 0.1, 1e-12);
}
TEST(WheelOdometry, InvalidSampleDoesNotCorruptState) {
  WheelOdometry odom(0.1, 0.4);
  odom.update(0, 0, 0.02);
  EXPECT_FALSE(odom.update(std::numeric_limits<double>::quiet_NaN(), 1, 0.1));
  EXPECT_FALSE(odom.update(1, 1, 0));
  EXPECT_TRUE(odom.update(1, 1, 0.1));
  EXPECT_NEAR(odom.x, 0.1, 1e-12);
}
TEST(CommandWatchdog, StaleFutureFrozenAndNonfiniteCommandsStop) {
  EXPECT_TRUE(command_is_fresh(1, 1.1, 0.1, 0.3, 0.5));
  EXPECT_FALSE(command_is_fresh(1, 1.31, 0.1, 0.3, 0.5));
  EXPECT_FALSE(command_is_fresh(1, 1, 0.31, 0.3, 0.5));
  EXPECT_FALSE(command_is_fresh(5, 1, 0.01, 0.3, 0.5));
  EXPECT_FALSE(command_is_fresh(1, 1, 0.01, 2, 0.5));
  EXPECT_FALSE(command_is_fresh(1, 1, 0.01, 0, std::numeric_limits<double>::infinity()));
}
