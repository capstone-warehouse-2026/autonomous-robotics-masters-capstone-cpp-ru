#pragma once

// Plain 2D grid types of the planner. No ROS dependency: the algorithms are unit-tested and
// benchmarked without a running graph, and the node only converts messages to these types.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace capstone_planning {

// Cell values of nav_msgs/OccupancyGrid (docs/INTERFACES.md): 0 = free, 100 = occupied, -1 = unknown.
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;
constexpr std::int8_t kUnknown = -1;

// Values in [0, 100] at or above this threshold are obstacles; negative values are unknown.
constexpr int kOccupiedThreshold = 50;

inline bool is_unknown_value(std::int8_t v) { return v < 0; }
inline bool is_occupied_value(std::int8_t v) { return v >= kOccupiedThreshold; }
inline bool is_free_value(std::int8_t v) { return v >= 0 && v < kOccupiedThreshold; }

/// Planar pose or rigid transform. As a transform `parent_from_child` it maps a point of the child
/// frame to the parent frame: p_parent = R(yaw) * p_child + (x, y). Units: m, rad.
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Point2D {
  double x{0.0};
  double y{0.0};
};

inline Point2D transform_point(const Pose2D &parent_from_child, const Point2D &p) {
  const double c = std::cos(parent_from_child.yaw);
  const double s = std::sin(parent_from_child.yaw);
  return {parent_from_child.x + c * p.x - s * p.y, parent_from_child.y + s * p.x + c * p.y};
}

inline Pose2D inverse(const Pose2D &t) {
  const double c = std::cos(t.yaw);
  const double s = std::sin(t.yaw);
  return {-(c * t.x + s * t.y), -(-s * t.x + c * t.y), -t.yaw};
}

/// a_from_c = a_from_b * b_from_c.
inline Pose2D compose(const Pose2D &a_from_b, const Pose2D &b_from_c) {
  const Point2D p = transform_point(a_from_b, {b_from_c.x, b_from_c.y});
  return {p.x, p.y, a_from_b.yaw + b_from_c.yaw};
}

struct Cell {
  int x{0};
  int y{0};
  bool operator==(const Cell &o) const { return x == o.x && y == o.y; }
  bool operator!=(const Cell &o) const { return !(*this == o); }
};

/// Occupancy grid with the layout of nav_msgs/OccupancyGrid: row-major, index y * width + x,
/// `origin` is the pose of the corner of cell (0, 0) in the grid's frame (yaw is allowed).
struct Grid {
  int width{0};
  int height{0};
  double resolution{0.05};  // m per cell
  Pose2D origin;
  std::vector<std::int8_t> data;

  bool valid() const {
    return width > 0 && height > 0 && std::isfinite(resolution) && resolution > 0.0 &&
           std::isfinite(origin.x) && std::isfinite(origin.y) && std::isfinite(origin.yaw) &&
           data.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
  std::size_t size() const { return data.size(); }
  bool contains(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
  bool contains(const Cell &c) const { return contains(c.x, c.y); }
  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
  }
  std::size_t index(const Cell &c) const { return index(c.x, c.y); }
  Cell cell_of(std::size_t i) const {
    return {static_cast<int>(i % static_cast<std::size_t>(width)), static_cast<int>(i / static_cast<std::size_t>(width))};
  }
  std::int8_t at(int x, int y) const { return data[index(x, y)]; }

  /// Cell that contains a point of the grid's frame. The cell may lie outside the grid: check contains().
  Cell cell_at(const Point2D &p) const {
    const Point2D local = transform_point(inverse(origin), p);
    return {static_cast<int>(std::floor(local.x / resolution)), static_cast<int>(std::floor(local.y / resolution))};
  }
  /// Point of the grid's frame at fractional cell coordinates (fx, fy); (x + 0.5, y + 0.5) is a cell centre.
  Point2D point_at(double fx, double fy) const { return transform_point(origin, {fx * resolution, fy * resolution}); }
  Point2D cell_center(const Cell &c) const { return point_at(c.x + 0.5, c.y + 0.5); }
};

}  // namespace capstone_planning
