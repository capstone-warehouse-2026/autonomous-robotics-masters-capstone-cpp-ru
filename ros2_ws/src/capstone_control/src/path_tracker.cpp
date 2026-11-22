#include "capstone_control/path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace capstone_control {

std::optional<std::string> validate_path(const Path &path, double max_gap) {
  if (path.empty()) {
    return std::string("path is empty");
  }
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!is_finite(path[i])) {
      std::ostringstream msg;
      msg << "pose " << i << " is not finite";
      return msg.str();
    }
    if (i > 0) {
      const double gap = distance(path[i - 1], path[i]);
      if (gap > max_gap) {
        std::ostringstream msg;
        msg << "gap of " << gap << " m between poses " << i - 1 << " and " << i << " exceeds " << max_gap
            << " m";
        return msg.str();
      }
    }
  }
  return std::nullopt;
}

PathTracker::PathTracker(Path path, double search_window)
    : path_(std::move(path)), search_window_(search_window) {
  cumulative_.reserve(path_.size());
  double s = 0.0;
  for (std::size_t i = 0; i < path_.size(); ++i) {
    if (i > 0) {
      s += distance(path_[i - 1], path_[i]);
    }
    cumulative_.push_back(s);
  }
}

void PathTracker::update(const Pose2D &robot) {
  if (path_.empty()) {
    return;
  }
  if (path_.size() == 1) {
    s_ = 0.0;
    cross_track_ = distance(path_.front(), robot);
    return;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::size_t best_segment = segment_;
  double best_s = s_;
  double best_cross = cross_track_;

  for (std::size_t i = segment_; i + 1 < path_.size(); ++i) {
    // Only segments that start within the search window ahead of the current projection.
    if (i > segment_ && cumulative_[i] > s_ + search_window_) {
      break;
    }
    const Pose2D &a = path_[i];
    const Pose2D &b = path_[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) {
      continue;  // duplicate pose
    }
    const double len = std::sqrt(len2);
    const double t = std::clamp(((robot.x - a.x) * dx + (robot.y - a.y) * dy) / len2, 0.0, 1.0);
    const double px = a.x + t * dx;
    const double py = a.y + t * dy;
    const double d = std::hypot(robot.x - px, robot.y - py);
    if (d < best_dist) {
      best_dist = d;
      best_segment = i;
      best_s = cumulative_[i] + t * len;
      // Sign from the 2D cross product: robot to the left of the segment is positive.
      const double cross = (dx * (robot.y - a.y) - dy * (robot.x - a.x)) / len;
      best_cross = std::copysign(d, cross);
    }
  }

  if (std::isfinite(best_dist)) {
    segment_ = best_segment;
    s_ = best_s;
    cross_track_ = best_cross;
  }
}

Pose2D PathTracker::point_ahead(double ahead) const {
  if (path_.size() <= 1) {
    return path_.empty() ? Pose2D{} : path_.front();
  }
  const double target = std::min(s_ + std::max(ahead, 0.0), total_length());
  if (target >= total_length()) {
    return path_.back();
  }
  // First pose whose arc length is beyond the target; the target lies on the segment before it.
  const auto it = std::upper_bound(cumulative_.begin(), cumulative_.end(), target);
  const std::size_t j = static_cast<std::size_t>(std::distance(cumulative_.begin(), it));
  const Pose2D &a = path_[j - 1];
  const Pose2D &b = path_[j];
  const double seg = cumulative_[j] - cumulative_[j - 1];
  const double t = seg > 1e-12 ? (target - cumulative_[j - 1]) / seg : 0.0;
  return Pose2D{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), std::atan2(b.y - a.y, b.x - a.x)};
}

}  // namespace capstone_control
