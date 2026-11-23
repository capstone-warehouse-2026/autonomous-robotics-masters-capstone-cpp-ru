#include "capstone_planning/grid_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>

namespace capstone_planning {

namespace {

constexpr double kSqrt2 = 1.4142135623730951;
constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr std::size_t kCheckEvery = 256;
constexpr std::int32_t kNoParent = -1;

struct OpenEntry {
  double f;
  double g;
  std::size_t index;
};

// Min-heap on f; among equal f prefer larger g (deeper node), then smaller index: deterministic order.
struct OpenGreater {
  bool operator()(const OpenEntry &a, const OpenEntry &b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g < b.g;
    return a.index > b.index;
  }
};

bool stop_requested(const SearchOptions &options, SearchResult &result) {
  if (options.should_cancel && options.should_cancel()) {
    result.status = PlanStatus::kCanceled;
    return true;
  }
  if (std::chrono::steady_clock::now() >= options.deadline) {
    result.status = PlanStatus::kTimeout;
    return true;
  }
  return false;
}

// Shortest (by length) route from a start in the inflated zone to the nearest traversable cell,
// through non-obstacle cells and within `radius`. Returns start .. escape cell, or empty.
std::vector<Cell> escape_route(const CostMap &map, const Cell &start, double radius, double &length) {
  const Grid &grid = map.grid();
  const double res = grid.resolution;
  std::vector<double> dist(grid.size(), std::numeric_limits<double>::infinity());
  std::vector<std::int32_t> parent(grid.size(), kNoParent);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const std::size_t s = grid.index(start);
  dist[s] = 0.0;
  open.push({0.0, s});
  while (!open.empty()) {
    const auto [d, i] = open.top();
    open.pop();
    if (d > dist[i]) continue;
    const Cell c = grid.cell_of(i);
    if (map.traversable(c)) {
      std::vector<Cell> route;
      for (std::int32_t k = static_cast<std::int32_t>(i); k != kNoParent; k = parent[k]) {
        route.push_back(grid.cell_of(static_cast<std::size_t>(k)));
      }
      std::reverse(route.begin(), route.end());
      length = d;
      return route;
    }
    for (int k = 0; k < 8; ++k) {
      const Cell n{c.x + kDx[k], c.y + kDy[k]};
      if (!grid.contains(n) || map.obstacle(n)) continue;
      const bool diagonal = kDx[k] != 0 && kDy[k] != 0;
      if (diagonal && (map.obstacle({c.x + kDx[k], c.y}) || map.obstacle({c.x, c.y + kDy[k]}))) continue;
      const double nd = d + (diagonal ? kSqrt2 : 1.0) * res;
      const std::size_t ni = grid.index(n);
      if (nd <= radius + 1e-9 && nd < dist[ni]) {
        dist[ni] = nd;
        parent[ni] = static_cast<std::int32_t>(i);
        open.push({nd, ni});
      }
    }
  }
  return {};
}

}  // namespace

const char *to_string(PlanStatus status) {
  switch (status) {
    case PlanStatus::kSuccess: return "success";
    case PlanStatus::kStartOutsideMap: return "start outside map";
    case PlanStatus::kGoalOutsideMap: return "goal outside map";
    case PlanStatus::kStartBlocked: return "start blocked";
    case PlanStatus::kGoalBlocked: return "goal blocked";
    case PlanStatus::kNoPath: return "no path";
    case PlanStatus::kCanceled: return "canceled";
    case PlanStatus::kTimeout: return "timeout";
  }
  return "unknown";
}

double step_cost(const CostMap &map, const Cell &from, const Cell &to) {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0) || !map.traversable(to)) {
    return CostMap::kBlocked;
  }
  const bool diagonal = dx != 0 && dy != 0;
  if (diagonal && (!map.traversable({from.x + dx, from.y}) || !map.traversable({from.x, from.y + dy}))) {
    return CostMap::kBlocked;  // no corner cutting
  }
  const double length = (diagonal ? kSqrt2 : 1.0) * map.resolution();
  return length * 0.5 * (map.cost(from) + map.cost(to));
}

double octile_heuristic(const CostMap &map, const Cell &a, const Cell &b) {
  const double dx = std::abs(a.x - b.x);
  const double dy = std::abs(a.y - b.y);
  return (std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy)) * map.resolution() * CostMap::kMinCostPerMeter;
}

SearchResult plan_on_grid(const CostMap &map, const Cell &start, const Cell &goal, const SearchOptions &options) {
  SearchResult result;
  const Grid &grid = map.grid();
  if (!grid.contains(start)) {
    result.status = PlanStatus::kStartOutsideMap;
    return result;
  }
  if (!grid.contains(goal)) {
    result.status = PlanStatus::kGoalOutsideMap;
    return result;
  }
  if (!map.traversable(goal)) {
    result.status = PlanStatus::kGoalBlocked;
    return result;
  }

  // Prefix from a start inside the hard mask to the nearest traversable cell.
  std::vector<Cell> prefix{start};
  double prefix_cost = 0.0;
  if (!map.traversable(start)) {
    double escape_length = 0.0;
    if (map.obstacle(start) || options.start_escape_radius <= 0.0 ||
        (prefix = escape_route(map, start, options.start_escape_radius, escape_length)).empty()) {
      result.status = PlanStatus::kStartBlocked;
      return result;
    }
    // Charged at the highest cost of the clearance term: leaving the mask is never "cheap".
    prefix_cost = escape_length * (CostMap::kMinCostPerMeter + map.params().clearance_weight);
  }
  const Cell search_start = prefix.back();

  const std::size_t n = grid.size();
  std::vector<double> g(n, std::numeric_limits<double>::infinity());
  std::vector<std::int32_t> parent(n, kNoParent);
  std::vector<std::uint8_t> closed(n, 0);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenGreater> open;

  const std::size_t s = grid.index(search_start);
  const std::size_t t = grid.index(goal);
  const auto h = [&](const Cell &c) { return options.use_heuristic ? octile_heuristic(map, c, goal) : 0.0; };
  g[s] = 0.0;
  open.push({h(search_start), 0.0, s});

  bool found = false;
  while (!open.empty()) {
    const OpenEntry top = open.top();
    open.pop();
    if (closed[top.index] || top.g > g[top.index]) {
      continue;  // stale entry
    }
    closed[top.index] = 1;
    if (top.index == t) {
      found = true;
      break;
    }
    ++result.expanded;
    if (options.max_expansions > 0 && result.expanded >= options.max_expansions) {
      result.status = PlanStatus::kTimeout;
      return result;
    }
    if (result.expanded % kCheckEvery == 0 && stop_requested(options, result)) {
      return result;
    }
    const Cell c = grid.cell_of(top.index);
    for (int k = 0; k < 8; ++k) {
      const Cell nb{c.x + kDx[k], c.y + kDy[k]};
      if (!grid.contains(nb)) continue;
      const std::size_t ni = grid.index(nb);
      if (closed[ni]) continue;
      const double step = step_cost(map, c, nb);
      if (step == CostMap::kBlocked) continue;
      const double ng = top.g + step;
      if (ng < g[ni]) {
        g[ni] = ng;
        parent[ni] = static_cast<std::int32_t>(top.index);
        open.push({ng + h(nb), ng, ni});
      }
    }
  }
  if (!found) {
    result.status = PlanStatus::kNoPath;
    return result;
  }

  std::vector<Cell> cells;
  for (std::int32_t k = static_cast<std::int32_t>(t); k != kNoParent; k = parent[k]) {
    cells.push_back(grid.cell_of(static_cast<std::size_t>(k)));
  }
  std::reverse(cells.begin(), cells.end());
  result.cells = prefix;
  result.cells.insert(result.cells.end(), cells.begin() + 1, cells.end());
  result.cost = prefix_cost + g[t];
  result.status = PlanStatus::kSuccess;
  return result;
}

}  // namespace capstone_planning
