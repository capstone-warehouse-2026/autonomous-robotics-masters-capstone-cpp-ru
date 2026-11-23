#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "capstone_planning/grid.hpp"
#include "capstone_planning/map_fusion.hpp"

namespace capstone_planning {

/// Fixed cost of method A. Robot geometry follows docs/adr/0002 (disc of 0.272 m); the same
/// footprint + margin is the hard collision mask for both methods (projects/03_planning.md).
struct CostParams {
  double robot_radius = 0.272;     // m
  double inflation_margin = 0.05;  // m added to the radius for the hard mask
  // Clearance cost: grows quadratically from 0 at clearance_band beyond the hard mask to
  // clearance_weight at its border.
  double clearance_band = 0.5;  // m
  double clearance_weight = 2.0;
  // Extra proximity cost of obstacles observed by perception but absent from /map (other robots,
  // people, moved pallets): the planner prefers a longer free route to a short congested one.
  double observed_band = 1.0;  // m
  double observed_weight = 3.0;
  // docs/INTERFACES.md: unknown blocks unless another mode is fixed before test.
  bool unknown_is_obstacle = true;
  double unknown_weight = 1.0;  // per-metre penalty of an unknown cell when it is not an obstacle

  double inflation_radius() const { return robot_radius + inflation_margin; }
  /// Throws std::invalid_argument for negative or non-finite values.
  void validate() const;
};

/// Hard collision mask and per-metre traversal cost of every cell.
///
/// Clearance is the distance from a cell centre to the nearest obstacle cell, minus half a cell
/// (the distance to the obstacle cell's edge); obstacle cells are occupied cells and, when
/// unknown_is_obstacle, unknown cells. Space outside the map is not an obstacle: the planner never
/// leaves the map anyway. A cell is traversable if it is not an obstacle and its clearance is at
/// least the inflation radius. The cost of a traversable cell is
///   1 + clearance_weight * s(clearance, clearance_band) + observed_weight * s(observed clearance,
///   observed_band) + unknown_weight * [unknown],
/// with s(d, band) = max(0, 1 - (d - inflation_radius) / band)^2. The cost is never below
/// kMinCostPerMeter, which keeps the octile heuristic admissible.
class CostMap {
 public:
  static constexpr double kMinCostPerMeter = 1.0;
  static constexpr double kBlocked = std::numeric_limits<double>::infinity();

  CostMap(const FusedMap &map, const CostParams &params);
  explicit CostMap(const Grid &map, const CostParams &params);

  const Grid &grid() const { return grid_; }
  const CostParams &params() const { return params_; }
  double resolution() const { return grid_.resolution; }

  bool traversable(const Cell &c) const { return grid_.contains(c) && cost_[grid_.index(c)] < kBlocked; }
  /// Obstacle cell itself (occupied, or unknown when unknown_is_obstacle).
  bool obstacle(const Cell &c) const { return grid_.contains(c) && obstacle_[grid_.index(c)] != 0; }
  /// Per-metre cost; kBlocked for a cell of the hard mask or outside the map.
  double cost(const Cell &c) const { return grid_.contains(c) ? cost_[grid_.index(c)] : kBlocked; }
  /// Distance to the nearest obstacle edge, m; +inf if the map has no obstacle.
  double clearance(const Cell &c) const { return clearance_[grid_.index(c)]; }

 private:
  void build(const std::vector<std::uint8_t> &observed_obstacle);

  Grid grid_;
  CostParams params_;
  std::vector<std::uint8_t> obstacle_;
  std::vector<double> clearance_;
  std::vector<double> cost_;
};

/// Exact squared Euclidean distance transform (Felzenszwalb & Huttenlocher, 2012) in cell units:
/// for every cell, the squared distance to the nearest cell with source != 0; a very large value if
/// there is no source.
std::vector<double> squared_distance_transform(const std::vector<std::uint8_t> &source, int width, int height);

}  // namespace capstone_planning
