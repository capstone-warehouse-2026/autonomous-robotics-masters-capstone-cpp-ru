#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "capstone_nav2/scan_grid.hpp"

namespace {
sensor_msgs::msg::LaserScan scan() {
  sensor_msgs::msg::LaserScan message;
  message.header.frame_id = "lidar_link";
  message.header.stamp.sec = 3;
  message.angle_min = 0;
  message.angle_increment = 0.1;
  message.range_min = 0.05;
  message.range_max = 12;
  message.ranges = {1.0F};
  return message;
}
int value(const nav_msgs::msg::OccupancyGrid &grid, double x, double y) {
  const int col = std::floor((x - grid.info.origin.position.x) / grid.info.resolution);
  const int row = std::floor((y - grid.info.origin.position.y) / grid.info.resolution);
  return grid.data.at(row * grid.info.width + col);
}
TEST(ScanGrid, TransformsHitAndKeepsMeasurementTimestamp) {
  auto grid = capstone_nav2::project_scan(scan(), {2, 3, 0}, {8, 0.1});
  EXPECT_EQ(grid.header.frame_id, "odom");
  EXPECT_EQ(grid.header.stamp.sec, 3);
  EXPECT_EQ(value(grid, 2.52, 3.02), 0);
  EXPECT_EQ(value(grid, 3.02, 3.02), 100);
  EXPECT_EQ(value(grid, 3.32, 3.02), -1);
}
TEST(ScanGrid, RotatesMeasuredObstacle) {
  auto grid = capstone_nav2::project_scan(scan(), {0, 0, std::acos(-1.0) / 2}, {8, 0.1});
  EXPECT_EQ(value(grid, 0.02, 1.02), 100);
  EXPECT_EQ(value(grid, 1.02, 0.02), -1);
}
TEST(ScanGrid, InfinityClearsRayAndNanDoesNotInventFreeSpace) {
  auto message = scan();
  message.ranges = {std::numeric_limits<float>::infinity()};
  auto grid = capstone_nav2::project_scan(message, {}, {8, 0.1});
  EXPECT_EQ(value(grid, 2.02, 0.02), 0);
  for (auto cell : grid.data) EXPECT_NE(cell, 100);
  message.ranges = {std::numeric_limits<float>::quiet_NaN()};
  grid = capstone_nav2::project_scan(message, {}, {8, 0.1});
  for (auto cell : grid.data) EXPECT_EQ(cell, -1);
}
TEST(ScanGrid, RejectsMalformedMetadataAndParameters) {
  auto message = scan();
  message.angle_increment = 0;
  EXPECT_THROW(capstone_nav2::project_scan(message, {}, {}), std::invalid_argument);
  EXPECT_THROW(capstone_nav2::project_scan(scan(), {}, {8, 0}), std::invalid_argument);
}
}  // namespace
