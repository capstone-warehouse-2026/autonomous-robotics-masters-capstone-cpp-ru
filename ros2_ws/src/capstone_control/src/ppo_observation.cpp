#include "capstone_control/ppo_observation.hpp"

#include <algorithm>
#include <cmath>

namespace capstone_control::ppo {

PathFeatures path_features(const ControlInput &input) {
  const PathTracker &path = *input.path;
  PathFeatures f;
  for (int i = 0; i < kPathPoints; ++i) {
    const Pose2D q = path.point_ahead((i + 1) * kPathSpacing);
    f.points[i] = to_robot_frame(input.pose, q.x, q.y);
  }
  f.remaining = path.remaining_length();
  f.goal_yaw = normalize_angle(path.goal().yaw - input.pose.yaw);
  return f;
}

std::vector<float> build_observation(const PathFeatures &path, const std::vector<Point2D> &obstacles,
                                     const Command &velocity) {
  std::vector<float> obs(kObsSize, 0.0f);
  for (int i = 0; i < kPathPoints; ++i) {
    obs[2 * i] = static_cast<float>(path.points[i].x / kPathScale);
    obs[2 * i + 1] = static_cast<float>(path.points[i].y / kPathScale);
  }
  obs[20] = static_cast<float>(std::min(path.remaining, kRemainingMax) / kRemainingMax);
  obs[21] = static_cast<float>(std::cos(path.goal_yaw));
  obs[22] = static_cast<float>(std::sin(path.goal_yaw));

  std::array<double, kSectors> ranges;
  ranges.fill(kRangeMax);
  const double width = 2.0 * kPi / kSectors;
  for (const auto &o : obstacles) {
    // A bearing of exactly +pi would give index kSectors; it belongs to the last sector.
    const int k = std::clamp(static_cast<int>(std::floor((std::atan2(o.y, o.x) + kPi) / width)), 0, kSectors - 1);
    ranges[k] = std::min(ranges[k], std::hypot(o.x, o.y));
  }
  for (int i = 0; i < kSectors; ++i) {
    obs[23 + i] = static_cast<float>(std::min(ranges[i], kRangeMax) / kRangeMax);
  }

  obs[59] = static_cast<float>(velocity.v / kVMax);
  obs[60] = static_cast<float>(velocity.w / kWMax);
  return obs;
}

std::vector<float> build_observation(const ControlInput &input) {
  return build_observation(path_features(input), input.obstacles, input.velocity);
}

Command action_to_target(float a0, float a1) {
  const double v = std::clamp(static_cast<double>(a0), -1.0, 1.0);
  const double w = std::clamp(static_cast<double>(a1), -1.0, 1.0);
  return Command{(v + 1.0) / 2.0 * kVMax, w * kWMax};
}

}  // namespace capstone_control::ppo