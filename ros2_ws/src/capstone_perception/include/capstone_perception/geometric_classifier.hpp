#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>

#include "capstone_perception/pixel_classifier.hpp"

namespace capstone_perception {

// Plane n·p + d = 0 with a unit normal n pointing up; height(p) is the signed distance of p over it.
struct FloorPlane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double offset = 0.0;
  double height(const Eigen::Vector3d &point) const { return normal.dot(point) + offset; }
};

// Method A, classical (participant 1): floor plane by RANSAC in base_link, height of every point over
// the floor, obstacle clusters, see projects/02_perception.md. Parameters are declared on the node
// under the "classical." prefix.
class GeometricClassifier : public PixelClassifier {
 public:
  explicit GeometricClassifier(rclcpp::Node &node);
  std::string name() const override { return "classical"; }
  cv::Mat classify(const RgbdFrame &frame) override;

  // Floor of the last classified frame and whether RANSAC found it (false: the fixed prior floor was used).
  const FloorPlane &last_floor() const { return last_floor_; }
  bool last_floor_found() const { return last_floor_found_; }
  // Flying pixels removed (made unknown) in the last classified frame.
  int last_flying_count() const { return last_flying_count_; }
  // Small obstacle clusters removed (made unknown) in the last classified frame.
  int last_removed_clusters() const { return last_removed_clusters_; }

 private:
  // RANSAC on floor candidates (points of base_link); returns false and leaves `floor` unchanged on failure.
  bool find_floor(const std::vector<Eigen::Vector3d> &candidates, FloorPlane &floor) const;
  // The accepted floor: normal close to "up" and height under the robot close to floor_z_.
  bool plausible_floor(const FloorPlane &floor) const;

  // Classification by height over the floor.
  double floor_z_;          // m, expected floor height in base_link (prior and fallback)
  double floor_tolerance_;  // m, |height over the floor| <= this -> free
  double robot_height_;     // m, higher than this -> the robot passes under it -> unknown
  // Floor plane search (RANSAC).
  bool ransac_enabled_;
  int ransac_iterations_;     // random plane hypotheses per frame
  int ransac_stride_;         // use every n-th pixel in both directions as a floor candidate
  double ransac_band_;        // m, candidates are points with |z - floor_z| <= band
  double ransac_threshold_;   // m, inlier distance to a hypothesis plane
  double ransac_max_offset_;  // m, accepted |floor height under the robot - floor_z|
  double ransac_min_cos_tilt_;  // cos of the largest accepted floor tilt
  int ransac_min_inliers_;    // fewer inliers -> RANSAC failed, fixed floor is used
  // Flying pixels at depth jumps.
  bool flying_enabled_;
  double flying_jump_ratio_;  // depth jump to both neighbours > max(min_jump, jump_ratio * z) -> flying
  double flying_min_jump_;    // m
  // Small obstacle clusters.
  bool clusters_enabled_;
  double clusters_min_area_;  // m², seen surface of a connected group of obstacle pixels; smaller -> unknown

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  FloorPlane last_floor_;
  bool last_floor_found_ = false;
  int last_flying_count_ = 0;
  int last_removed_clusters_ = 0;
};

}  // namespace capstone_perception
