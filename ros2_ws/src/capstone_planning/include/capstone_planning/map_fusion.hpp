#pragma once

#include <cstdint>
#include <vector>

#include "capstone_planning/grid.hpp"

namespace capstone_planning {

struct FusionOptions {
  // A cell that is unknown in the static map but observed free by perception becomes free.
  bool observed_free_clears_unknown = true;
};

/// Static map with the latest local obstacle grid on top, in the frame and geometry of the static map.
struct FusedMap {
  Grid grid;
  // 1 where the cell is occupied only because perception observes an obstacle there (not in /map):
  // the "observed occupancy" that gets an extra proximity cost.
  std::vector<std::uint8_t> observed_obstacle;
};

/// Fuses an un-inflated local grid (G2 /perception/obstacles, normally in `odom`) into the static map.
/// `map_from_local` is the pose of the local grid's frame in the map frame (TF map <- odom).
///
/// Rules, applied without memory between calls, so an obstacle that left the local window or moved
/// disappears from the next fused map:
/// - obstacles of the static map are never cleared (perception does not erase walls);
/// - a cell observed occupied becomes occupied. Each occupied local cell is projected with 3x3 sample
///   points, so a thin obstacle is not lost when the two grids are rotated or have different resolution;
/// - a cell observed free clears only an unknown static cell (FusionOptions);
/// - cells outside the local window keep their static value.
/// Throws std::invalid_argument for an invalid grid.
FusedMap fuse_maps(const Grid &static_map, const Grid *local, const Pose2D &map_from_local,
                   const FusionOptions &options = {});

}  // namespace capstone_planning
