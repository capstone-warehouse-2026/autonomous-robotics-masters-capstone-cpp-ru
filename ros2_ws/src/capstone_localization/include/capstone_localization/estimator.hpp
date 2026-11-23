#pragma once

#include <array>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace capstone_localization {

using Pose = Eigen::Vector3d;  // (x, y, yaw), metres and radians.
using Covariance = Eigen::Matrix3d;
using Points = std::vector<Eigen::Vector2d>;

double wrap_angle(double radians);
Pose compose(const Pose &parent_from_frame, const Pose &frame_from_child);
Pose between(const Pose &world_from_parent, const Pose &world_from_child);

struct Filter {
  Pose pose = Pose::Zero();
  Covariance covariance = Covariance::Zero();

  // Increment in the frame of the robot at the start of the step.
  void predict(const Pose &increment, const Covariance &process_noise);
  double nis(const Pose &measurement, const Covariance &measurement_noise) const;
  void correct(const Pose &measurement, const Covariance &measurement_noise);
};

struct Match {
  Pose pose = Pose::Zero();
  double rms = 0.0;
  double inlier_fraction = 0.0;
  double inlier_count = 0.0;
  double geometry_ratio = 0.0;
};

// A regular occupancy grid indexes cell faces facing known free space.
class ScanMatcher {
 public:
  void set_map(unsigned width, unsigned height, double resolution, const Pose &origin,
               const std::vector<signed char> &data);
  bool ready() const { return width_ != 0; }
  std::optional<Match> match(const Points &scan, const Pose &prediction, double search_radius) const;

 private:
  struct Surface {
    Eigen::Vector2d center;
    Eigen::Vector2d normal;
  };

  const Surface *nearest(const Eigen::Vector2d &point, double radius) const;
  unsigned width_ = 0;
  unsigned height_ = 0;
  double resolution_ = 0.0;
  Pose origin_ = Pose::Zero();
  std::vector<std::vector<Surface>> surfaces_;
};

}  // namespace capstone_localization
