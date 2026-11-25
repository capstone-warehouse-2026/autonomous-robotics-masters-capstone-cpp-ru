#include "capstone_integration/path_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace capstone_integration {

namespace {

std::size_t nearest_index(const std::vector<Pose2D> &path, const Pose2D &pose) {
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = distance(path[i], pose);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  return best;
}

void validate_follower(const FollowerParams &p) {
  if (!(p.lookahead_m > 0.0) || !(p.max_v > 0.0) || !(p.max_w > 0.0) ||
      !(p.heading_gain > 0.0) || !(p.turn_in_place_angle_rad > 0.0) ||
      !(p.arrive_radius_m > 0.0)) {
    throw std::invalid_argument("follow_path: follower parameters must be positive");
  }
}

}  // namespace

std::vector<Pose2D> straight_path(const Pose2D &start, const Pose2D &goal, double step_m) {
  if (!(step_m > 0.0) || !std::isfinite(step_m)) {
    throw std::invalid_argument("straight_path: step_m must be positive and finite");
  }
  const double total = distance(start, goal);
  std::vector<Pose2D> path;
  if (total < 1e-9) {
    path.push_back(Pose2D{goal.x, goal.y, goal.yaw});
    return path;
  }
  const auto steps = static_cast<std::size_t>(std::ceil(total / step_m));
  const double segment_yaw = std::atan2(goal.y - start.y, goal.x - start.x);
  path.reserve(steps + 1);
  for (std::size_t i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    path.push_back(Pose2D{start.x + (goal.x - start.x) * t, start.y + (goal.y - start.y) * t,
                          i == steps ? goal.yaw : segment_yaw});
  }
  path.back() = Pose2D{goal.x, goal.y, goal.yaw};
  return path;
}

double path_length(const std::vector<Pose2D> &path) {
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += distance(path[i - 1], path[i]);
  }
  return total;
}

double remaining_distance(const std::vector<Pose2D> &path, const Pose2D &pose) {
  if (path.empty()) {
    return 0.0;
  }
  const std::size_t from = nearest_index(path, pose);
  return std::max(0.0, path_length(std::vector<Pose2D>(path.begin() + static_cast<long>(from),
                                                       path.end())));
}

ControlCommand follow_path(const std::vector<Pose2D> &path, const Pose2D &pose,
                           const FollowerParams &params) {
  validate_follower(params);
  if (path.empty()) {
    return ControlCommand{};
  }
  if (distance(path.back(), pose) <= params.arrive_radius_m) {
    return ControlCommand{};
  }

  const std::size_t from = nearest_index(path, pose);
  Pose2D target = path.back();
  double travelled = 0.0;
  for (std::size_t i = from + 1; i < path.size(); ++i) {
    travelled += distance(path[i - 1], path[i]);
    target = path[i];
    if (travelled >= params.lookahead_m) {
      break;
    }
  }

  const double desired_heading = std::atan2(target.y - pose.y, target.x - pose.x);
  const double error = angle_diff(desired_heading, pose.yaw);

  ControlCommand command;
  if (std::abs(error) >= params.turn_in_place_angle_rad) {
    command.v = 0.0;
  } else {
    const double slowdown = 1.0 - 0.7 * std::abs(error) / params.turn_in_place_angle_rad;
    command.v = clamp(params.max_v * slowdown, 0.0, params.max_v);
  }
  command.w = clamp(params.heading_gain * error, -params.max_w, params.max_w);
  return command;
}

}  // namespace capstone_integration
