// Offline benchmark of method A on the synthetic G3 scenarios (no ROS graph needed):
//   ros2 run capstone_planning planning_benchmark [repetitions]
// Prints a Markdown table: status, latency p50/p95/p99 (cost map + search), expanded nodes, path
// length, min / 5th percentile clearance, and the A* vs Dijkstra cost check on the same cost.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/map_fusion.hpp"
#include "capstone_planning/planner.hpp"
#include "capstone_planning/scenarios.hpp"

namespace {

using namespace capstone_planning;

double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double pos = q * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = std::min(v.size() - 1, lo + 1);
  return v[lo] + (pos - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

}  // namespace

int main(int argc, char **argv) {
  const int repetitions = argc > 1 ? std::max(1, std::atoi(argv[1])) : 50;
  const CostParams params;
  std::printf("Method A (classical), %d repetitions, robot_radius %.3f m + margin %.3f m\n\n", repetitions,
              params.robot_radius, params.inflation_margin);
  std::printf("| scenario | status | expected path | total ms p50 / p95 / p99 | cost map ms p50 | A* expanded | "
              "Dijkstra expanded | A* = Dijkstra cost | length, m | min / p5 clearance, m |\n");
  std::printf("|---|---|---|---|---|---|---|---|---|---|\n");
  int failures = 0;
  for (const auto &s : scenarios::all()) {
    std::vector<double> total_ms, map_ms;
    PlannedPath astar, dijkstra;
    for (int r = 0; r < repetitions; ++r) {
      const auto t0 = std::chrono::steady_clock::now();
      const FusedMap fused = fuse_maps(s.static_map, s.local.width > 0 ? &s.local : nullptr, s.map_from_local);
      const CostMap map(fused, params);
      const auto t1 = std::chrono::steady_clock::now();
      astar = plan_path(map, s.start, s.goal);
      const auto t2 = std::chrono::steady_clock::now();
      map_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
      total_ms.push_back(std::chrono::duration<double, std::milli>(t2 - t0).count());
      if (r == 0) {
        SearchOptions reference;
        reference.use_heuristic = false;
        dijkstra = plan_path(map, s.start, s.goal, reference);
      }
    }
    const bool ok = (astar.status == PlanStatus::kSuccess) == s.expect_path;
    const bool same_cost = astar.status == dijkstra.status &&
                           (astar.status != PlanStatus::kSuccess || std::abs(astar.cost - dijkstra.cost) < 1e-9);
    failures += (ok && same_cost) ? 0 : 1;
    const bool success = astar.status == PlanStatus::kSuccess;
    std::printf("| %s | %s | %s | %.2f / %.2f / %.2f | %.2f | %zu | %zu | %s | %s | %s |\n", s.name.c_str(),
                to_string(astar.status), s.expect_path ? "yes" : "no", percentile(total_ms, 0.5),
                percentile(total_ms, 0.95), percentile(total_ms, 0.99), percentile(map_ms, 0.5), astar.expanded,
                dijkstra.expanded, same_cost ? "yes" : "NO",
                success ? std::to_string(astar.length).substr(0, 5).c_str() : "-",
                success ? (std::to_string(astar.min_clearance).substr(0, 5) + " / " +
                           std::to_string(astar.p5_clearance).substr(0, 5))
                              .c_str()
                        : "-");
  }
  return failures == 0 ? 0 : 1;
}
