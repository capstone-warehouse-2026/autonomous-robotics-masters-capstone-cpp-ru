#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "capstone_control/geometry.hpp"

namespace capstone_control {

/// Returns an error message if the path cannot be followed, std::nullopt if it is valid.
/// Rules (ADR 0004): at least one pose, all values finite, consecutive poses no more
/// than `max_gap` metres apart. The frame (`map`) is checked by the ROS layer.
std::optional<std::string> validate_path(const Path &path, double max_gap);

/// Tracks the robot's progress along a fixed path.
///
/// The path is treated as a polyline through the pose positions; only the yaw of the
/// last pose is used (the goal heading). The projection point moves forward only:
/// it is searched from the current segment up to `search_window` metres ahead, so
/// a path that loops back near itself cannot make the robot "jump" backwards.
class PathTracker {
 public:
  PathTracker() = default;
  /// The path must already have passed validate_path().
  PathTracker(Path path, double search_window);

  /// Updates the projection of the robot position onto the path.
  void update(const Pose2D &robot);

  /// Point on the path `ahead` metres past the current projection (clamped to the end).
  Pose2D point_ahead(double ahead) const;

  const Pose2D &goal() const { return path_.back(); }
  double total_length() const { return cumulative_.empty() ? 0.0 : cumulative_.back(); }
  /// Arc length from the start to the current projection.
  double progress() const { return s_; }
  /// Arc length left from the current projection to the goal.
  double remaining_length() const { return total_length() - s_; }
  /// Signed distance from the path at the last update (left of the path is positive).
  double cross_track_error() const { return cross_track_; }
  bool empty() const { return path_.empty(); }

 private:
  Path path_;
  std::vector<double> cumulative_;  // arc length at each pose
  double search_window_{1.5};
  std::size_t segment_{0};  // index of the segment start that holds the projection
  double s_{0.0};
  double cross_track_{0.0};
};

}  // namespace capstone_control
