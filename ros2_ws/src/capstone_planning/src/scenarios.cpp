#include "capstone_planning/scenarios.hpp"

#include <cmath>

namespace capstone_planning::scenarios {

namespace {
constexpr double kResolution = 0.05;
}

Grid make_grid(double width_m, double height_m, double resolution, std::int8_t value) {
  Grid grid;
  grid.resolution = resolution;
  grid.width = static_cast<int>(std::lround(width_m / resolution));
  grid.height = static_cast<int>(std::lround(height_m / resolution));
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, value);
  return grid;
}

void fill_rect(Grid &grid, double x0, double y0, double x1, double y1, std::int8_t value) {
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const Point2D c = grid.cell_center({x, y});
      if (c.x >= x0 && c.x <= x1 && c.y >= y0 && c.y <= y1) {
        grid.data[grid.index(x, y)] = value;
      }
    }
  }
}

void add_border(Grid &grid) {
  for (int x = 0; x < grid.width; ++x) {
    grid.data[grid.index(x, 0)] = kOccupied;
    grid.data[grid.index(x, grid.height - 1)] = kOccupied;
  }
  for (int y = 0; y < grid.height; ++y) {
    grid.data[grid.index(0, y)] = kOccupied;
    grid.data[grid.index(grid.width - 1, y)] = kOccupied;
  }
}

Scenario narrow_passage(double gap_width) {
  Scenario s;
  s.name = "narrow_passage_" + std::to_string(static_cast<int>(std::lround(gap_width * 100))) + "cm";
  s.static_map = make_grid(8.0, 4.0, kResolution);
  add_border(s.static_map);
  fill_rect(s.static_map, 3.9, 0.0, 4.1, 2.0 - 0.5 * gap_width);
  fill_rect(s.static_map, 3.9, 2.0 + 0.5 * gap_width, 4.1, 4.0);
  s.start = {1.0, 2.0, 0.0};
  s.goal = {7.0, 2.0, 0.0};
  // The hard mask needs a gap of at least 2 * (0.272 + 0.05) m plus cell rounding.
  s.expect_path = gap_width >= 0.75;
  return s;
}

Scenario two_routes(bool block_a, bool crowd_a, bool block_b) {
  Scenario s;
  s.name = std::string("two_routes") + (block_a ? "_blocked_a" : "") + (crowd_a ? "_crowded_a" : "") +
           (block_b ? "_closed_b" : "");
  s.static_map = make_grid(10.0, 6.0, kResolution);
  add_border(s.static_map);
  fill_rect(s.static_map, 4.9, 0.0, 5.1, 2.2);  // wall below gap A
  fill_rect(s.static_map, 4.9, 3.8, 5.1, 4.4);  // wall between gaps A and B
  if (block_b) {
    fill_rect(s.static_map, 4.9, 4.4, 5.1, 6.0);
  }
  s.start = {1.0, 3.0, 0.0};
  s.goal = {9.0, 3.0, 0.0};
  s.expect_path = !(block_a && block_b);

  if (block_a || crowd_a) {
    // Local grid of perception in odom: 6 x 6 m window, odom rotated by 0.3 rad and shifted in map,
    // so the scenario also exercises the transform of map fusion.
    s.map_from_local = {2.0, 0.5, 0.3};
    s.local = make_grid(6.0, 6.0, kResolution, kFree);
    const auto mark = [&](double x0, double y0, double x1, double y1) {
      // Obstacles are described in map coordinates and written to the local grid cells they cover.
      for (int y = 0; y < s.local.height; ++y) {
        for (int x = 0; x < s.local.width; ++x) {
          const Point2D m = transform_point(s.map_from_local, s.local.cell_center({x, y}));
          if (m.x >= x0 && m.x <= x1 && m.y >= y0 && m.y <= y1) {
            s.local.data[s.local.index(x, y)] = kOccupied;
          }
        }
      }
    };
    if (block_a) {
      mark(5.15, 2.1, 5.45, 3.9);  // e.g. a pallet left in gap A
    }
    if (crowd_a) {
      for (double x : {3.0, 3.6, 4.2, 5.8, 6.4, 7.0}) {
        mark(x - 0.1, 2.2, x + 0.1, 2.3);  // people / robots 0.7 m from the line y = 3
        mark(x - 0.1, 3.7, x + 0.1, 3.8);
      }
    }
  }
  return s;
}

Scenario unreachable_goal() {
  Scenario s;
  s.name = "unreachable_goal";
  s.static_map = make_grid(8.0, 4.0, kResolution);
  add_border(s.static_map);
  fill_rect(s.static_map, 5.0, 1.0, 7.0, 1.1);
  fill_rect(s.static_map, 5.0, 2.9, 7.0, 3.0);
  fill_rect(s.static_map, 5.0, 1.0, 5.1, 3.0);
  fill_rect(s.static_map, 6.9, 1.0, 7.0, 3.0);
  s.start = {1.0, 2.0, 0.0};
  s.goal = {6.0, 2.0, 0.0};
  s.expect_path = false;
  return s;
}

Scenario warehouse_like() {
  Scenario s;
  s.name = "warehouse_like_20x20m";
  s.static_map = make_grid(20.0, 20.0, kResolution);
  s.static_map.origin = {-10.0, -10.0, 0.0};
  add_border(s.static_map);
  // Four rack rows 1 m deep with 2.1 m aisles and a cross aisle at x = -1 m (docs/adr/0002, approximate).
  for (double yc : {-4.65, -1.55, 1.55, 4.65}) {
    fill_rect(s.static_map, -6.0, yc - 0.5, -1.6, yc + 0.5);
    fill_rect(s.static_map, -0.4, yc - 0.5, 6.0, yc + 0.5);
  }
  fill_rect(s.static_map, 7.0, -8.0, 9.0, -6.0);  // pallet zone
  s.start = {-8.0, 0.0, 0.0};
  s.goal = {3.0, 6.6, 1.5707963267948966};  // between the top rack row and the wall
  return s;
}

std::vector<Scenario> all() {
  return {narrow_passage(1.0),
          narrow_passage(0.5),
          two_routes(false, false),
          two_routes(true, false),
          two_routes(false, true),
          two_routes(true, false, true),
          unreachable_goal(),
          warehouse_like()};
}

}  // namespace capstone_planning::scenarios
