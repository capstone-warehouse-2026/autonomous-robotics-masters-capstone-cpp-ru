#include <gtest/gtest.h>
#include "capstone_system_test/route_evidence.hpp"
using namespace capstone_system_test;
TEST(RouteEvidence, SparseSegmentThroughObstacleIsRejected) {
  const Rectangle obstacle{-0.5, -2, 0.5, 2};
  EXPECT_TRUE(intersects(obstacle, {-3, 0}, {3, 0}));
  EXPECT_FALSE(valid_detour({{-3, 0}, {-1, 0}, {3, 0}}, obstacle));
}
TEST(RouteEvidence, InflatedShelfRequiresRealDetour) {
  const auto obstacle = inflate({-0.5, -2, 0.5, 2}, 0.35);
  EXPECT_TRUE(valid_detour({{-3, 0}, {-1, -2.6}, {1, -2.6}, {3, 0}}, obstacle));
  EXPECT_FALSE(valid_detour({{-3, 0}, {-1, -2.1}, {1, -2.1}, {3, 0}}, obstacle));
}
TEST(RouteEvidence, OrdinaryLongPathDoesNotProveAvoidance) {
  EXPECT_FALSE(valid_detour({{-3, -3}, {-3, -4}, {3, -4}, {3, -3}}, {-0.5, -2, 0.5, 2}));
  EXPECT_FALSE(intersects({-0.5, -2, 0.5, 2}, {-3, -3}, {3, -3}));
}
TEST(RouteEvidence, TangencyAndDegenerateSegmentsAreConservative) {
  EXPECT_TRUE(intersects({0, 0, 1, 1}, {-1, 1}, {2, 1}));
  EXPECT_TRUE(intersects({0, 0, 1, 1}, {0.5, 0.5}, {0.5, 0.5}));
  EXPECT_FALSE(intersects({0, 0, 1, 1}, {2, 2}, {2, 2}));
}

TEST(RouteEvidence, ServiceNeedsNearbyPhysicalSample) {
  const std::vector<PositionSample> samples{{1.0, {2.0, 3.0}}, {1.1, {2.1, 3.0}}};
  const auto position = nearest_position(samples, 1.09, 0.15);
  ASSERT_TRUE(position);
  EXPECT_DOUBLE_EQ(position->x, 2.1);
  EXPECT_FALSE(nearest_position(samples, 4.0, 0.15));
  EXPECT_FALSE(nearest_position({}, 1.0, 0.15));
}
TEST(RouteEvidence, EventLogQuotesDoNotChangeColumns) {
  const auto row = csv_row("episode,job,1.2,delivered,\"reason,with comma\",target");
  ASSERT_EQ(row.size(), 6u);
  EXPECT_EQ(row[4], "reason,with comma");
  EXPECT_THROW(csv_row("\"broken"), std::invalid_argument);
}
