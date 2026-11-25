#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/grid.hpp"
#include "capstone_planning/grid_search.hpp"

namespace capstone_planning {

/// Path of method A in the frame of the cost map (`map`), with the metrics of projects/03_planning.md.
struct PlannedPath {
  PlanStatus status = PlanStatus::kNoPath;
  std::vector<Pose2D> poses;  // exact start, cell centres, exact goal; empty unless kSuccess
  double cost = std::numeric_limits<double>::infinity();  // internal cost; not comparable across methods
  std::size_t expanded = 0;
  double length = 0.0;          // m
  double min_clearance = 0.0;   // m, distance from the path to the nearest obstacle edge
  double p5_clearance = 0.0;    // m, 5th percentile over path samples
  double planning_ms = 0.0;     // steady clock, search and path conversion (not the cost map)
};

/// Plans from `start` to `goal` (poses in the cost map frame). The first pose is the exact start,
/// the last one the exact goal with its yaw; intermediate poses are cell centres oriented to the
/// next pose (G4 ignores intermediate yaw, ADR 0004).
PlannedPath plan_path(const CostMap &map, const Pose2D &start, const Pose2D &goal, const SearchOptions &options = {});

/// Converts a cell path to poses (see plan_path).
std::vector<Pose2D> cells_to_poses(const Grid &grid, const std::vector<Cell> &cells, const Pose2D &start,
                                   const Pose2D &goal);

double path_length(const std::vector<Pose2D> &poses);

/// Points every half cell along the polyline, endpoints included.
std::vector<Point2D> sample_path(const std::vector<Pose2D> &poses, double step);

struct ClearanceStats {
  double min = std::numeric_limits<double>::infinity();
  double p5 = std::numeric_limits<double>::infinity();
};
ClearanceStats path_clearance(const CostMap &map, const std::vector<Pose2D> &poses);

/// True if the path, after its first traversable sample (the robot may start inside the hard mask),
/// crosses a cell that is no longer traversable or leaves the map: the trigger for replanning when
/// the map is updated.
bool path_blocked(const CostMap &map, const std::vector<Pose2D> &poses);

}  // namespace capstone_planning
