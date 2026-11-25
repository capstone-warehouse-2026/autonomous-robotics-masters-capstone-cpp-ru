#include "capstone_planning/map_fusion.hpp"

#include <stdexcept>

namespace capstone_planning {

FusedMap fuse_maps(const Grid &static_map, const Grid *local, const Pose2D &map_from_local,
                   const FusionOptions &options) {
  if (!static_map.valid()) {
    throw std::invalid_argument("fuse_maps: invalid static map");
  }
  FusedMap fused;
  fused.grid = static_map;
  fused.observed_obstacle.assign(static_map.size(), 0);
  fused.observed_area.assign(static_map.size(), 0);
  if (local == nullptr) {
    return fused;
  }
  if (!local->valid()) {
    throw std::invalid_argument("fuse_maps: invalid local grid");
  }

  auto mark_observed_obstacle = [&](std::size_t i) {
    if (!is_occupied_value(static_map.data[i])) {
      fused.grid.data[i] = kOccupied;
      fused.observed_obstacle[i] = 1;
    }
  };

  // Pass 1: every map cell centre looks up the local cell under it (free and occupied observations).
  const Pose2D local_from_map = inverse(map_from_local);
  for (int y = 0; y < static_map.height; ++y) {
    for (int x = 0; x < static_map.width; ++x) {
      const Point2D in_local = transform_point(local_from_map, static_map.cell_center({x, y}));
      const Cell lc = local->cell_at(in_local);
      if (!local->contains(lc)) {
        continue;
      }
      const std::int8_t observed = local->data[local->index(lc)];
      const std::size_t i = static_map.index(x, y);
      fused.observed_area[i] = 1;
      if (is_occupied_value(observed)) {
        mark_observed_obstacle(i);
      } else if (is_free_value(observed) && options.observed_free_clears_unknown &&
                 is_unknown_value(static_map.data[i])) {
        fused.grid.data[i] = kFree;
      }
    }
  }

  // Pass 2: forward projection of occupied local cells, so obstacles smaller than a map cell survive.
  static constexpr double kSamples[3] = {0.1, 0.5, 0.9};
  for (int y = 0; y < local->height; ++y) {
    for (int x = 0; x < local->width; ++x) {
      if (!is_occupied_value(local->at(x, y))) {
        continue;
      }
      for (double sy : kSamples) {
        for (double sx : kSamples) {
          const Point2D in_map = transform_point(map_from_local, local->point_at(x + sx, y + sy));
          const Cell mc = static_map.cell_at(in_map);
          if (static_map.contains(mc)) {
            mark_observed_obstacle(static_map.index(mc));
          }
        }
      }
    }
  }
  return fused;
}

}  // namespace capstone_planning
