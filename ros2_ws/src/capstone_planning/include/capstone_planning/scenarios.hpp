#pragma once

// Synthetic maps of the G3 experiment scenarios (projects/03_planning.md), shared by the tests and
// the planning_benchmark executable. Maps are in `map`, resolution 0.05 m, origin (0, 0).

#include <string>
#include <vector>

#include "capstone_planning/grid.hpp"

namespace capstone_planning::scenarios {

struct Scenario {
  std::string name;
  Grid static_map;
  // Local "perception" grid in its own frame (odom) and its pose in map; width 0 = no local grid.
  Grid local;
  Pose2D map_from_local;
  Pose2D start;
  Pose2D goal;
  bool expect_path = true;
};

/// Empty grid with every cell set to `value`.
Grid make_grid(double width_m, double height_m, double resolution, std::int8_t value = kFree);
/// Sets every cell whose centre lies in [x0, x1] x [y0, y1] (grid frame) to `value`.
void fill_rect(Grid &grid, double x0, double y0, double x1, double y1, std::int8_t value = kOccupied);
/// Occupied border one cell thick.
void add_border(Grid &grid);

/// 8 x 4 m room split by a wall at x = 4 m with one gap of `gap_width` m centred at y = 2 m.
Scenario narrow_passage(double gap_width);

/// 10 x 6 m room split by a wall at x = 5 m with gap A (y 2.2..3.8, on the straight line) and gap B
/// (y 4.4..5.9, a detour). `block_a`: perception observes an obstacle across gap A (corridor blocked);
/// `crowd_a`: perception observes small obstacles 0.7 m on both sides of the line near gap A
/// (short but congested route versus the longer free route B); `block_b` closes gap B in the static map.
Scenario two_routes(bool block_a, bool crowd_a, bool block_b = false);

/// Goal inside a closed box: unreachable, the planner must report no path.
Scenario unreachable_goal();

/// 20 x 20 m map shaped like the simulated warehouse (rack rows, aisles, cross aisle), origin
/// (-10, -10): planning latency at full map size. An approximation, not the exported Webots map.
Scenario warehouse_like();

/// All scenarios of the benchmark.
std::vector<Scenario> all();

}  // namespace capstone_planning::scenarios
