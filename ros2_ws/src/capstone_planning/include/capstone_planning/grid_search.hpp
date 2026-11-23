#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/grid.hpp"

namespace capstone_planning {

enum class PlanStatus {
  kSuccess,
  kStartOutsideMap,
  kGoalOutsideMap,
  kStartBlocked,  // start cell is an obstacle, or is in the hard mask with no free cell within the escape radius
  kGoalBlocked,   // goal cell is an obstacle or in the hard mask
  kNoPath,        // the search space is exhausted: the goal is unreachable
  kCanceled,
  kTimeout,       // deadline or expansion limit reached
};

const char *to_string(PlanStatus status);

struct SearchOptions {
  // false: Dijkstra with the same cost and neighbourhood (correctness reference for A*).
  bool use_heuristic = true;
  // Planning deadline (steady clock); the default never expires.
  std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max();
  std::size_t max_expansions = 0;  // 0 = unlimited
  // Checked every few hundred expansions; returning true stops the search with kCanceled.
  std::function<bool()> should_cancel;
  // The robot may start inside the inflated zone (pushed by localization noise or a new obstacle):
  // then the path first leaves it along the shortest route through non-obstacle cells, at most this
  // far. 0 disables the escape and reports kStartBlocked.
  double start_escape_radius = 0.5;  // m
};

struct SearchResult {
  PlanStatus status = PlanStatus::kNoPath;
  std::vector<Cell> cells;  // start .. goal, 8-connected, empty unless kSuccess
  double cost = std::numeric_limits<double>::infinity();
  std::size_t expanded = 0;  // nodes taken from the open list and expanded
};

/// Cost of one 8-connected step: step length (m) times the mean per-metre cost of its two cells.
/// Diagonal steps are allowed only if both side cells are traversable (no corner cutting);
/// otherwise and for a blocked target cell the cost is CostMap::kBlocked.
double step_cost(const CostMap &map, const Cell &from, const Cell &to);

/// Octile distance (m) times CostMap::kMinCostPerMeter. Every step costs at least its length times
/// that bound, so the heuristic is admissible and consistent for this cost.
double octile_heuristic(const CostMap &map, const Cell &a, const Cell &b);

/// A* (or Dijkstra) on the 8-connected grid of the cost map from start to goal.
SearchResult plan_on_grid(const CostMap &map, const Cell &start, const Cell &goal, const SearchOptions &options = {});

}  // namespace capstone_planning
