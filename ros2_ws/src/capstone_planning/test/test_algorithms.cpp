// Pure C++ tests of method A: map fusion, inflation/cost, A* search, path conversion. No ROS context.

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/grid_search.hpp"
#include "capstone_planning/map_fusion.hpp"
#include "capstone_planning/planner.hpp"
#include "capstone_planning/scenarios.hpp"

namespace {

using namespace capstone_planning;
using scenarios::add_border;
using scenarios::fill_rect;
using scenarios::make_grid;

CostParams tiny_robot() {
  // Inflation radius below one cell: only obstacle cells are blocked; for topology tests.
  CostParams p;
  p.robot_radius = 0.01;
  p.inflation_margin = 0.0;
  p.clearance_weight = 0.0;
  p.observed_weight = 0.0;
  return p;
}

PlannedPath plan_scenario(const scenarios::Scenario &s, const CostParams &params = {}) {
  const FusedMap fused = fuse_maps(s.static_map, s.local.width > 0 ? &s.local : nullptr, s.map_from_local);
  return plan_path(CostMap(fused, params), s.start, s.goal);
}

// --- grid and transforms ---------------------------------------------------------------------

TEST(Grid, CellLookupWithRotatedOrigin) {
  Grid g = make_grid(1.0, 1.0, 0.1);
  g.origin = {2.0, 1.0, M_PI / 2};  // x of the grid points along +y of the frame
  const Point2D c = g.cell_center({3, 1});
  EXPECT_NEAR(c.x, 2.0 - 0.15, 1e-12);
  EXPECT_NEAR(c.y, 1.0 + 0.35, 1e-12);
  EXPECT_EQ(g.cell_at(c), (Cell{3, 1}));
  EXPECT_FALSE(g.contains(g.cell_at({0.0, 0.0})));
}

TEST(Grid, ComposeAndInverse) {
  const Pose2D a{1.0, -2.0, 0.7};
  const Pose2D identity = compose(a, inverse(a));
  EXPECT_NEAR(identity.x, 0.0, 1e-12);
  EXPECT_NEAR(identity.y, 0.0, 1e-12);
  EXPECT_NEAR(identity.yaw, 0.0, 1e-12);
}

// --- map fusion --------------------------------------------------------------------------------

TEST(MapFusion, ObservedObstacleLandsInMapCellThroughTransform) {
  Grid map = make_grid(10.0, 10.0, 0.05);
  Grid local = make_grid(4.0, 4.0, 0.05);
  local.origin = {-2.0, -2.0, 0.0};  // local window centred on odom
  // Obstacle cell with centre (1.025, 0.025) in odom.
  local.data[local.index(local.cell_at({1.01, 0.01}))] = kOccupied;
  const Pose2D map_from_odom{5.0, 5.0, M_PI / 2};  // odom (1.025, 0.025) -> map (4.975, 6.025)
  const FusedMap fused = fuse_maps(map, &local, map_from_odom);
  const Cell expected = map.cell_at({4.975, 6.025});
  EXPECT_EQ(fused.grid.data[map.index(expected)], kOccupied);
  EXPECT_EQ(fused.observed_obstacle[map.index(expected)], 1);
  // Far away cells untouched.
  EXPECT_EQ(fused.grid.data[map.index(map.cell_at({1.0, 1.0}))], kFree);
}

TEST(MapFusion, StaticObstaclesStayAndFreeClearsOnlyUnknown) {
  Grid map = make_grid(4.0, 4.0, 0.1, kUnknown);
  fill_rect(map, 1.0, 1.0, 1.2, 3.0);  // static wall inside the local window
  Grid local = make_grid(4.0, 4.0, 0.1, kFree);  // perception sees everything free
  const FusedMap fused = fuse_maps(map, &local, {});
  EXPECT_EQ(fused.grid.data[map.index(map.cell_at({1.1, 2.0}))], kOccupied);  // wall kept
  EXPECT_EQ(fused.grid.data[map.index(map.cell_at({3.0, 3.0}))], kFree);      // unknown cleared
  EXPECT_EQ(fused.observed_obstacle[map.index(map.cell_at({1.1, 2.0}))], 0);  // static, not observed

  FusionOptions keep_unknown;
  keep_unknown.observed_free_clears_unknown = false;
  EXPECT_EQ(fuse_maps(map, &local, {}, keep_unknown).grid.data[map.index(map.cell_at({3.0, 3.0}))], kUnknown);
}

TEST(MapFusion, ThinObstacleSurvivesCoarseRotatedMap) {
  Grid map = make_grid(4.0, 4.0, 0.2);
  Grid local = make_grid(2.0, 2.0, 0.05);
  local.data[local.index(10, 10)] = kOccupied;  // one 5 cm cell
  const FusedMap fused = fuse_maps(map, &local, {1.0, 1.0, 0.4});
  int occupied = 0;
  for (auto v : fused.grid.data) occupied += v == kOccupied;
  EXPECT_GE(occupied, 1);
}

TEST(MapFusion, NoMemoryBetweenUpdates) {
  // An obstacle that moved away disappears: fusion always starts from the static map.
  Grid map = make_grid(4.0, 4.0, 0.1);
  Grid local = make_grid(4.0, 4.0, 0.1);
  local.data[local.index(5, 5)] = kOccupied;
  ASSERT_EQ(fuse_maps(map, &local, {}).grid.data[map.index(5, 5)], kOccupied);
  local.data[local.index(5, 5)] = kFree;
  EXPECT_EQ(fuse_maps(map, &local, {}).grid.data[map.index(5, 5)], kFree);
}

TEST(MapFusion, RejectsInvalidGrid) {
  Grid map = make_grid(1.0, 1.0, 0.1);
  Grid bad = map;
  bad.data.pop_back();
  EXPECT_THROW(fuse_maps(bad, nullptr, {}), std::invalid_argument);
  EXPECT_THROW(fuse_maps(map, &bad, {}), std::invalid_argument);
}

// --- cost map ----------------------------------------------------------------------------------

TEST(CostMap, DistanceTransformMatchesBruteForce) {
  std::mt19937 rng(7);
  const int w = 23, h = 17;
  std::vector<std::uint8_t> src(w * h, 0);
  for (auto &s : src) s = (rng() % 13 == 0) ? 1 : 0;
  const auto d2 = squared_distance_transform(src, w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      double best = 1e30;
      for (int j = 0; j < h; ++j)
        for (int i = 0; i < w; ++i)
          if (src[j * w + i]) best = std::min(best, double((x - i) * (x - i) + (y - j) * (y - j)));
      EXPECT_DOUBLE_EQ(d2[y * w + x], best) << x << "," << y;
    }
  }
}

TEST(CostMap, InflationAndClearanceCost) {
  Grid g = make_grid(4.0, 4.0, 0.05);
  g.data[g.index(40, 40)] = kOccupied;
  CostParams p;  // inflation radius 0.322 m
  const CostMap map(g, p);
  EXPECT_TRUE(map.obstacle({40, 40}));
  EXPECT_FALSE(map.traversable({40, 40}));
  // 6 cells away: clearance 6 * 0.05 - 0.025 = 0.275 < 0.322 -> blocked; 8 cells: 0.375 -> free.
  EXPECT_NEAR(map.clearance({46, 40}), 0.275, 1e-9);
  EXPECT_FALSE(map.traversable({46, 40}));
  EXPECT_TRUE(map.traversable({48, 40}));
  // Cost decreases with clearance and reaches the lower bound outside the band.
  EXPECT_GT(map.cost({48, 40}), map.cost({52, 40}));
  EXPECT_DOUBLE_EQ(map.cost({70, 40}), CostMap::kMinCostPerMeter);
  for (int y = 0; y < g.height; ++y)
    for (int x = 0; x < g.width; ++x) EXPECT_GE(map.cost({x, y}), CostMap::kMinCostPerMeter);
}

TEST(CostMap, UnknownBlocksByDefault) {
  Grid g = make_grid(2.0, 2.0, 0.05, kUnknown);
  EXPECT_FALSE(CostMap(g, tiny_robot()).traversable({20, 20}));
  CostParams allow = tiny_robot();
  allow.unknown_is_obstacle = false;
  const CostMap map(g, allow);
  EXPECT_TRUE(map.traversable({20, 20}));
  EXPECT_DOUBLE_EQ(map.cost({20, 20}), 1.0 + allow.unknown_weight);
}

TEST(CostMap, RejectsInvalidParams) {
  Grid g = make_grid(1.0, 1.0, 0.05);
  CostParams p;
  p.robot_radius = 0.0;
  EXPECT_THROW(CostMap(g, p), std::invalid_argument);
  p = CostParams{};
  p.clearance_weight = std::nan("");
  EXPECT_THROW(CostMap(g, p), std::invalid_argument);
}

// --- search ------------------------------------------------------------------------------------

TEST(Search, StraightAndDiagonalOnEmptyMap) {
  const CostMap map(make_grid(2.0, 2.0, 0.1), tiny_robot());
  auto r = plan_on_grid(map, {0, 0}, {9, 0});
  ASSERT_EQ(r.status, PlanStatus::kSuccess);
  EXPECT_EQ(r.cells.size(), 10u);
  EXPECT_NEAR(r.cost, 0.9, 1e-9);
  r = plan_on_grid(map, {0, 0}, {5, 5});
  ASSERT_EQ(r.status, PlanStatus::kSuccess);
  EXPECT_EQ(r.cells.size(), 6u);
  EXPECT_NEAR(r.cost, 5 * std::sqrt(2.0) * 0.1, 1e-9);
  r = plan_on_grid(map, {3, 3}, {3, 3});
  ASSERT_EQ(r.status, PlanStatus::kSuccess);
  EXPECT_EQ(r.cells.size(), 1u);
}

TEST(Search, NoCornerCutting) {
  Grid g = make_grid(0.3, 0.3, 0.1);
  g.data[g.index(1, 0)] = kOccupied;
  g.data[g.index(0, 1)] = kOccupied;
  const CostMap map(g, tiny_robot());
  EXPECT_EQ(step_cost(map, {0, 0}, {1, 1}), CostMap::kBlocked);
  EXPECT_EQ(plan_on_grid(map, {0, 0}, {1, 1}).status, PlanStatus::kNoPath);
  // One side open is still corner cutting.
  g.data[g.index(0, 1)] = kFree;
  const CostMap map2(g, tiny_robot());
  EXPECT_EQ(step_cost(map2, {0, 0}, {1, 1}), CostMap::kBlocked);
  const auto r = plan_on_grid(map2, {0, 0}, {1, 1});
  ASSERT_EQ(r.status, PlanStatus::kSuccess);
  EXPECT_EQ(r.cells.size(), 3u);  // (0,0) -> (0,1) -> (1,1)
}

TEST(Search, ExplicitFailures) {
  Grid g = make_grid(4.0, 4.0, 0.05);
  add_border(g);
  fill_rect(g, 2.0, 0.0, 2.1, 4.0);  // wall splits the map
  const CostMap map(g, CostParams{});
  EXPECT_EQ(plan_on_grid(map, {-1, 5}, {10, 10}).status, PlanStatus::kStartOutsideMap);
  EXPECT_EQ(plan_on_grid(map, {10, 10}, {100, 10}).status, PlanStatus::kGoalOutsideMap);
  EXPECT_EQ(plan_on_grid(map, {20, 20}, {41, 20}).status, PlanStatus::kGoalBlocked);   // goal in the wall
  EXPECT_EQ(plan_on_grid(map, {20, 20}, {36, 20}).status, PlanStatus::kGoalBlocked);   // goal in the inflation
  EXPECT_EQ(plan_on_grid(map, {41, 20}, {20, 20}).status, PlanStatus::kStartBlocked);  // start in the wall
  const auto r = plan_on_grid(map, {20, 20}, {60, 20});
  EXPECT_EQ(r.status, PlanStatus::kNoPath);
  EXPECT_TRUE(r.cells.empty());
  EXPECT_GT(r.expanded, 0u);
}

TEST(Search, StartInsideInflationEscapes) {
  Grid g = make_grid(4.0, 4.0, 0.05);
  fill_rect(g, 0.0, 0.0, 4.0, 0.1);  // wall along the bottom
  const CostMap map(g, CostParams{});
  const Cell start = g.cell_at({1.0, 0.3});  // 0.2 m from the wall: inside the hard mask
  ASSERT_FALSE(map.traversable(start));
  ASSERT_FALSE(map.obstacle(start));
  const auto r = plan_on_grid(map, start, g.cell_at({3.0, 2.0}));
  ASSERT_EQ(r.status, PlanStatus::kSuccess);
  EXPECT_EQ(r.cells.front(), start);
  SearchOptions no_escape;
  no_escape.start_escape_radius = 0.0;
  EXPECT_EQ(plan_on_grid(map, start, g.cell_at({3.0, 2.0}), no_escape).status, PlanStatus::kStartBlocked);
}

TEST(Search, AStarMatchesDijkstraOnRandomMaps) {
  std::mt19937 rng(42);
  for (int trial = 0; trial < 60; ++trial) {
    Grid g = make_grid(2.0, 2.0, 0.05);  // 40 x 40
    for (auto &v : g.data) {
      const auto r = rng() % 100;
      v = r < 18 ? kOccupied : (r < 21 ? kUnknown : kFree);
    }
    CostParams p;
    p.robot_radius = 0.03;
    p.inflation_margin = 0.0;
    p.clearance_band = 0.2;
    p.unknown_is_obstacle = trial % 2 == 0;
    const CostMap map(g, p);
    std::uniform_int_distribution<int> coord(0, 39);
    for (int q = 0; q < 5; ++q) {
      const Cell s{coord(rng), coord(rng)};
      const Cell t{coord(rng), coord(rng)};
      SearchOptions dijkstra;
      dijkstra.use_heuristic = false;
      dijkstra.start_escape_radius = 0.0;
      SearchOptions astar = dijkstra;
      astar.use_heuristic = true;
      const auto a = plan_on_grid(map, s, t, astar);
      const auto d = plan_on_grid(map, s, t, dijkstra);
      ASSERT_EQ(a.status, d.status);
      if (a.status != PlanStatus::kSuccess) continue;
      EXPECT_NEAR(a.cost, d.cost, 1e-9);
      EXPECT_LE(a.expanded, d.expanded);
      // Admissibility of the heuristic for this cost.
      EXPECT_LE(octile_heuristic(map, s, t), d.cost + 1e-12);
      // Recomputed cost of the returned path equals the reported cost; every step is legal.
      double sum = 0.0;
      for (std::size_t i = 1; i < a.cells.size(); ++i) {
        const double step = step_cost(map, a.cells[i - 1], a.cells[i]);
        ASSERT_NE(step, CostMap::kBlocked);
        sum += step;
      }
      EXPECT_NEAR(sum, a.cost, 1e-9);
    }
  }
}

TEST(Search, CancelTimeoutAndExpansionLimit) {
  const CostMap map(make_grid(20.0, 20.0, 0.05), tiny_robot());
  SearchOptions cancel;
  cancel.use_heuristic = false;
  cancel.should_cancel = [] { return true; };
  EXPECT_EQ(plan_on_grid(map, {0, 0}, {399, 399}, cancel).status, PlanStatus::kCanceled);

  SearchOptions deadline;
  deadline.use_heuristic = false;
  deadline.deadline = std::chrono::steady_clock::now();
  EXPECT_EQ(plan_on_grid(map, {0, 0}, {399, 399}, deadline).status, PlanStatus::kTimeout);

  SearchOptions limit;
  limit.max_expansions = 100;
  limit.use_heuristic = false;
  const auto r = plan_on_grid(map, {0, 0}, {399, 399}, limit);
  EXPECT_EQ(r.status, PlanStatus::kTimeout);
  EXPECT_EQ(r.expanded, 100u);
}

// --- scenarios of projects/03_planning.md ------------------------------------------------------

TEST(Scenarios, NarrowPassage) {
  const auto wide = plan_scenario(scenarios::narrow_passage(1.0));
  ASSERT_EQ(wide.status, PlanStatus::kSuccess);
  EXPECT_GE(wide.min_clearance, CostParams{}.inflation_radius() - 1e-9);
  EXPECT_EQ(plan_scenario(scenarios::narrow_passage(0.5)).status, PlanStatus::kNoPath);
}

TEST(Scenarios, BlockedCorridorDetours) {
  const auto free_a = plan_scenario(scenarios::two_routes(false, false));
  const auto blocked_a = plan_scenario(scenarios::two_routes(true, false));
  ASSERT_EQ(free_a.status, PlanStatus::kSuccess);
  ASSERT_EQ(blocked_a.status, PlanStatus::kSuccess);
  EXPECT_NEAR(free_a.length, 8.0, 0.1);  // straight through gap A
  EXPECT_GT(blocked_a.length, free_a.length + 1.0);
  bool through_b = false;
  for (const auto &p : blocked_a.poses) through_b = through_b || (std::abs(p.x - 5.0) < 0.1 && p.y > 4.4);
  EXPECT_TRUE(through_b);
  EXPECT_EQ(plan_scenario(scenarios::two_routes(true, false, true)).status, PlanStatus::kNoPath);
}

TEST(Scenarios, CongestedShortRouteLosesToFreeLongRoute) {
  const auto crowded = plan_scenario(scenarios::two_routes(false, true));
  ASSERT_EQ(crowded.status, PlanStatus::kSuccess);
  EXPECT_GT(crowded.length, 9.0);  // via gap B
  // Without the observed-occupancy term the crowd is passable and the short route wins.
  CostParams no_observed;
  no_observed.observed_weight = 0.0;
  const auto ignoring = plan_scenario(scenarios::two_routes(false, true), no_observed);
  ASSERT_EQ(ignoring.status, PlanStatus::kSuccess);
  EXPECT_LT(ignoring.length, 8.5);
}

TEST(Scenarios, UnreachableGoalIsExplicitFailure) {
  const auto r = plan_scenario(scenarios::unreachable_goal());
  EXPECT_EQ(r.status, PlanStatus::kNoPath);
  EXPECT_TRUE(r.poses.empty());
}

// --- path conversion and replanning trigger ---------------------------------------------------

TEST(Path, ExactEndpointsAndGoalYaw) {
  const CostMap map(make_grid(4.0, 4.0, 0.05), tiny_robot());
  const Pose2D start{0.51, 0.52, 1.0};
  const Pose2D goal{3.03, 2.98, -2.0};
  const auto path = plan_path(map, start, goal);
  ASSERT_EQ(path.status, PlanStatus::kSuccess);
  EXPECT_DOUBLE_EQ(path.poses.front().x, start.x);
  EXPECT_DOUBLE_EQ(path.poses.front().y, start.y);
  EXPECT_DOUBLE_EQ(path.poses.back().x, goal.x);
  EXPECT_DOUBLE_EQ(path.poses.back().yaw, goal.yaw);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    // Adjacent poses at most one diagonal cell apart (G4 max_path_gap is 1 m).
    EXPECT_LE(std::hypot(path.poses[i].x - path.poses[i - 1].x, path.poses[i].y - path.poses[i - 1].y), 0.1);
  }
  EXPECT_NEAR(path.length, path_length(path.poses), 1e-12);
}

TEST(Path, BlockedAfterMapUpdate) {
  const auto s = scenarios::two_routes(false, false);
  const CostMap before(s.static_map, CostParams{});
  const auto path = plan_path(before, s.start, s.goal);
  ASSERT_EQ(path.status, PlanStatus::kSuccess);
  EXPECT_FALSE(path_blocked(before, path.poses));
  const auto blocked = scenarios::two_routes(true, false);
  const CostMap after(fuse_maps(blocked.static_map, &blocked.local, blocked.map_from_local), CostParams{});
  EXPECT_TRUE(path_blocked(after, path.poses));
}

}  // namespace
