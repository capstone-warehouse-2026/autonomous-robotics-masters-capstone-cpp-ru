#include "capstone_fleet/planner.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <tuple>

#include <gtest/gtest.h>

namespace capstone_fleet {
namespace {
Grid open_grid(int width, int height) {
  return {width, height, 1.0, 0.0, 0.0,
          std::vector<std::uint8_t>(static_cast<std::size_t>(width) * height, 0), {}};
}

void verify_solution(const Grid &grid, const std::vector<Cell> &starts,
                     const std::vector<Cell> &goals, const PlanResult &result, double clearance) {
  ASSERT_EQ(result.status, PlanStatus::Success) << result.reason;
  ASSERT_EQ(result.paths.size(), starts.size());
  int cost = 0;
  for (std::size_t agent = 0; agent < starts.size(); ++agent) {
    const auto &path = result.paths[agent];
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), starts[agent]);
    EXPECT_EQ(path.back(), goals[agent]);
    EXPECT_EQ(path.size(), result.paths.front().size());
    for (std::size_t time = 1; time < path.size(); ++time)
      EXPECT_TRUE(grid.traversable(path[time - 1], path[time]));
    std::size_t arrival = path.size() - 1;
    while (arrival > 0 && path[arrival - 1] == goals[agent]) --arrival;
    cost += static_cast<int>(arrival);
  }
  EXPECT_EQ(cost, result.cost);
  EXPECT_TRUE(find_conflicts(grid, result.paths, clearance).empty());
  EXPECT_EQ(result.final_conflicts, 0U);
  EXPECT_LE(result.lower_bound, result.cost);
  EXPECT_LE(result.cost, result.suboptimality_bound * result.lower_bound + 1e-9);
}

// Независимый Dijkstra по совместным состояниям двух роботов служит оракулом
// оптимальной суммы arrival times. Флаг finished фиксирует вечное ожидание цели;
// пока флаг не выбран, робот вправе пройти через цель и покинуть её снова.
int joint_optimum(const Grid &grid, const std::vector<Cell> &starts,
                  const std::vector<Cell> &goals, double clearance) {
  using State = std::tuple<int, int, int>;
  const auto encode = [&](Cell cell) { return cell.y * grid.width + cell.x; };
  const auto decode = [&](int cell) { return Cell{cell % grid.width, cell / grid.width}; };
  const State initial{encode(starts[0]), encode(starts[1]), 0};
  std::map<State, int> distance{{initial, 0}};
  std::priority_queue<std::pair<int, State>, std::vector<std::pair<int, State>>,
                       std::greater<std::pair<int, State>>> open;
  open.push({0, initial});
  const auto relax = [&](State state, int cost, auto &queue, auto &distances) {
    const auto found = distances.find(state);
    if (found != distances.end() && found->second <= cost) return;
    distances[state] = cost;
    queue.push({cost, state});
  };
  const std::array<Cell, 5> moves{{{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  while (!open.empty()) {
    const auto [cost, state] = open.top();
    open.pop();
    if (distance[state] != cost) continue;
    const auto [first, second, finished] = state;
    if (finished == 3) return cost;
    const Cell a = decode(first), b = decode(second);
    if (!(finished & 1) && a == goals[0]) relax({first, second, finished | 1}, cost, open, distance);
    if (!(finished & 2) && b == goals[1]) relax({first, second, finished | 2}, cost, open, distance);
    const int step_cost = ((finished & 1) ? 0 : 1) + ((finished & 2) ? 0 : 1);
    for (const auto first_move : moves) {
      const Cell next_a{a.x + first_move.x, a.y + first_move.y};
      if ((finished & 1) && next_a != a) continue;
      if (!grid.traversable(a, next_a)) continue;
      for (const auto second_move : moves) {
        const Cell next_b{b.x + second_move.x, b.y + second_move.y};
        if ((finished & 2) && next_b != b) continue;
        if (!grid.traversable(b, next_b) ||
            segment_distance(grid, a, next_a, b, next_b) + 1e-9 < clearance) continue;
        relax({encode(next_a), encode(next_b), finished}, cost + step_cost, open, distance);
      }
    }
  }
  return -1;
}
}  // namespace

TEST(FleetGeometry, DetectsVertexSwapFollowingAndCrossingButNotSeparatedEdges) {
  const auto grid = open_grid(4, 4);
  EXPECT_FALSE(find_conflicts(grid, {{{0, 0}, {1, 0}}, {{2, 0}, {1, 0}}}, 0.85).empty());
  const auto swap = find_conflicts(grid, {{{0, 0}, {1, 0}}, {{1, 0}, {0, 0}}}, 0.85);
  ASSERT_EQ(swap.size(), 1U);
  EXPECT_EQ(swap.front().kind, ConflictKind::SweptEdge);
  EXPECT_FALSE(find_conflicts(grid, {{{0, 0}, {1, 0}}, {{1, 0}, {2, 0}}}, 0.85).empty());
  EXPECT_NEAR(segment_distance(grid, {0, 1}, {2, 1}, {1, 0}, {1, 2}), 0.0, 1e-12);
  EXPECT_NEAR(segment_distance(grid, {0, 0}, {1, 0}, {0, 1}, {1, 1}), 1.0, 1e-12);
  EXPECT_TRUE(find_conflicts(grid, {{{0, 0}, {1, 0}}, {{0, 1}, {1, 1}}}, 0.85).empty());
}

TEST(FleetGeometry, GoalOccupancyPersistsAfterShorterPathEnds) {
  const auto grid = open_grid(4, 2);
  const auto conflicts = find_conflicts(grid, {{{1, 0}}, {{3, 0}, {2, 0}, {1, 0}}}, 0.85);
  ASSERT_FALSE(conflicts.empty());
  EXPECT_EQ(conflicts.back().kind, ConflictKind::Vertex);
  EXPECT_EQ(conflicts.back().time, 2);
}

TEST(FleetGrid, BlockedEdgesAreUndirectedAndDistanceUsesStaticObstacles) {
  auto grid = open_grid(3, 2);
  grid.blocked_edges.push_back({{0, 0}, {1, 0}});
  EXPECT_FALSE(grid.traversable({0, 0}, {1, 0}));
  EXPECT_FALSE(grid.traversable({1, 0}, {0, 0}));
  EXPECT_EQ(shortest_distance(grid, {0, 0}, {2, 0}), 4);
  grid.blocked[3] = 1;
  EXPECT_EQ(shortest_distance(grid, {0, 0}, {2, 0}), -1);
}

TEST(FleetPlanner, CbsResolvesHeadOnSwapByActualDetour) {
  auto grid = open_grid(11, 9);
  grid.origin_x = -5;
  grid.origin_y = -4;
  for (int y = 2; y <= 6; ++y) grid.blocked[y * grid.width + 5] = 1;
  const std::vector<Cell> starts{{2, 1}, {8, 1}}, goals{{8, 1}, {2, 1}};
  PlanOptions options;
  const auto result = plan(grid, starts, goals, options);
  verify_solution(grid, starts, goals, result, options.clearance);
  EXPECT_FALSE(result.initial_conflicts.empty());
  EXPECT_FALSE(result.resolved_conflicts.empty());
  EXPECT_GT(result.high_level_expansions, 1U);
  bool changed_row = false;
  for (const auto &path : result.paths)
    for (const auto cell : path) changed_row |= cell.y != 1;
  EXPECT_TRUE(changed_row);
  options.method = Method::ECBS;
  const auto enhanced = plan(grid, starts, goals, options);
  verify_solution(grid, starts, goals, enhanced, options.clearance);
  EXPECT_GT(enhanced.low_level_focal_slack_expansions, 0U);
  EXPECT_GT(enhanced.high_level_focal_slack_expansions, 0U);
  EXPECT_LE(enhanced.cost, options.weight * result.cost);
}

TEST(FleetPlanner, CbsMatchesJointDijkstraAndEcbsRespectsItsBound) {
  const auto grid = open_grid(3, 2);
  const auto cell = [](int index) { return Cell{index % 3, index / 3}; };
  int checked = 0;
  // Все 30 упорядоченных пар начал × 30 пар целей, включая начало на своей цели.
  for (int a = 0; a < 6; ++a) for (int b = 0; b < 6; ++b) {
    if (a == b) continue;
    for (int c = 0; c < 6; ++c) for (int d = 0; d < 6; ++d) {
      if (c == d) continue;
      const std::vector<Cell> starts{cell(a), cell(b)}, goals{cell(c), cell(d)};
      const int optimum = joint_optimum(grid, starts, goals, 0.85);
      ASSERT_GE(optimum, 0);
      PlanOptions options;
      options.max_time_steps = 20;
      const auto cbs = plan(grid, starts, goals, options);
      verify_solution(grid, starts, goals, cbs, options.clearance);
      EXPECT_EQ(cbs.cost, optimum);
      for (const double weight : {1.1, 1.5, 2.0}) {
        options.method = Method::ECBS;
        options.weight = weight;
        const auto ecbs = plan(grid, starts, goals, options);
        verify_solution(grid, starts, goals, ecbs, options.clearance);
        EXPECT_LE(ecbs.lower_bound, optimum);
        EXPECT_LE(ecbs.cost, weight * optimum + 1e-9);
      }
      ++checked;
    }
  }
  EXPECT_EQ(checked, 900);
}

TEST(FleetPlanner, AllowsGoalToBeVacatedBeforeFinalArrival) {
  auto grid = open_grid(3, 2);
  grid.blocked[3] = grid.blocked[5] = 1;
  const std::vector<Cell> starts{{1, 0}, {0, 0}}, goals{{1, 0}, {2, 0}};
  const auto result = plan(grid, starts, goals);
  verify_solution(grid, starts, goals, result, 0.85);
  EXPECT_EQ(result.cost, joint_optimum(grid, starts, goals, 0.85));
  EXPECT_TRUE(std::any_of(result.paths[0].begin(), result.paths[0].end(),
                          [](Cell cell) { return cell.y == 1; }));
}

TEST(FleetPlanner, DistinguishesInfeasibleHorizonAndResourceLimits) {
  const auto grid = open_grid(3, 1);
  PlanOptions options;
  options.max_time_steps = 3;
  auto result = plan(grid, {{0, 0}, {2, 0}}, {{2, 0}, {0, 0}}, options);
  EXPECT_EQ(result.status, PlanStatus::NoSolution);
  EXPECT_TRUE(result.paths.empty());
  options.max_low_level_expansions = 1;
  result = plan(grid, {{0, 0}}, {{2, 0}}, options);
  EXPECT_EQ(result.status, PlanStatus::ResourceLimit);
  EXPECT_TRUE(result.paths.empty());
}

TEST(FleetPlanner, RejectsInvalidInputsAndImpossibleSharedGoals) {
  const auto grid = open_grid(3, 2);
  EXPECT_EQ(plan(grid, {{0, 0}, {0, 0}}, {{1, 0}, {2, 0}}).status, PlanStatus::InvalidInput);
  EXPECT_EQ(plan(grid, {{0, 0}, {2, 0}}, {{1, 0}, {1, 0}}).status, PlanStatus::NoSolution);
  PlanOptions options;
  options.weight = 0.9;
  EXPECT_EQ(plan(grid, {{0, 0}}, {{2, 0}}, options).status, PlanStatus::InvalidInput);
}

TEST(FleetAssignment, FindsGlobalMinimumInsteadOfGreedyRobotChoices) {
  const auto result = minimum_cost_assignment({{1, 2}, {2, 100}});
  ASSERT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(result.total_cost, 4);
  EXPECT_EQ(result.assignments[0].job, 1U);
  EXPECT_EQ(result.assignments[1].job, 0U);
}

TEST(FleetAssignment, MaximizesCardinalityAndNeverDuplicatesAJob) {
  const auto result = minimum_cost_assignment({{0, 1, -1}, {0, -1, -1}, {-1, -1, 2}});
  ASSERT_EQ(result.assignments.size(), 3U);
  EXPECT_EQ(result.total_cost, 3);
  std::set<std::size_t> robots, jobs;
  for (const auto &assignment : result.assignments) {
    EXPECT_TRUE(robots.insert(assignment.robot).second);
    EXPECT_TRUE(jobs.insert(assignment.job).second);
  }
}

TEST(FleetAssignment, LeavesUnreachableJobsUnassignedAndValidatesMatrix) {
  const auto result = minimum_cost_assignment({{-1, 7}, {-1, -1}});
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments.front().robot, 0U);
  EXPECT_EQ(result.assignments.front().job, 1U);
  EXPECT_TRUE(minimum_cost_assignment({}).assignments.empty());
  EXPECT_THROW(minimum_cost_assignment({{1, 2}, {1}}), std::invalid_argument);
}

TEST(FleetAssignment, MatchesExhaustiveOracleOnRectangularAndUnreachableMatrices) {
  unsigned random = 2026;
  for (int example = 0; example < 50; ++example) {
    std::vector<std::vector<int>> costs(3, std::vector<int>(4));
    for (auto &row : costs) {
      for (auto &cost : row) {
        random = 1664525U * random + 1013904223U;
        cost = static_cast<int>((random >> 16) % 12) - 2;
      }
    }
    int best_count = -1, best_cost = INT_MAX;
    std::function<void(int, unsigned, int, int)> enumerate;
    enumerate = [&](int robot, unsigned taken, int count, int cost) {
      if (robot == 3) {
        if (count > best_count || (count == best_count && cost < best_cost)) {
          best_count = count;
          best_cost = cost;
        }
        return;
      }
      enumerate(robot + 1, taken, count, cost);
      for (unsigned job = 0; job < 4; ++job)
        if (!(taken & (1U << job)) && costs[robot][job] >= 0)
          enumerate(robot + 1, taken | (1U << job), count + 1, cost + costs[robot][job]);
    };
    enumerate(0, 0, 0, 0);
    const auto actual = minimum_cost_assignment(costs);
    EXPECT_EQ(actual.assignments.size(), static_cast<std::size_t>(best_count));
    EXPECT_EQ(actual.total_cost, best_cost);
  }
}
}  // namespace capstone_fleet
