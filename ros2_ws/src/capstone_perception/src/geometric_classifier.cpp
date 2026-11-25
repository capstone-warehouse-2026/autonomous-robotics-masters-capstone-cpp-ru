#include "capstone_perception/geometric_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace capstone_perception {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Plane through three points with the normal turned up; false if the points are (almost) on one line.
bool plane_from_points(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c,
                       FloorPlane &plane) {
  Eigen::Vector3d normal = (b - a).cross(c - a);
  const double length = normal.norm();
  if (length < 1e-9) {
    return false;
  }
  normal /= length;
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  plane.normal = normal;
  plane.offset = -normal.dot(a);
  return true;
}

// Least squares plane through the points: it passes through their centroid, and its normal is the
// direction of the smallest spread (eigenvector of the covariance with the smallest eigenvalue).
FloorPlane fit_plane(const std::vector<Eigen::Vector3d> &points) {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto &p : points) {
    centroid += p;
  }
  centroid /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &p : points) {
    const Eigen::Vector3d q = p - centroid;
    covariance += q * q.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  Eigen::Vector3d normal = solver.eigenvectors().col(0);  // eigenvalues are sorted in increasing order
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  FloorPlane plane;
  plane.normal = normal;
  plane.offset = -normal.dot(centroid);
  return plane;
}

// Depth of a neighbour "far" from z: no measurement, or a jump larger than the threshold.
bool far_from(float z, float neighbour, double threshold) {
  return !std::isfinite(neighbour) || std::abs(neighbour - z) > threshold;
}

// Flying (mixed) pixels: at the border of an object the depth of a pixel is a mix of the object and of
// the background behind it, so its point hangs in the air between them. Such a pixel is far in depth
// from BOTH its neighbours on one line (left and right, or above and below). A pixel at the border of a
// real surface has at least one neighbour of the same surface, so it is kept. Returns a CV_8UC1 mask,
// 255 = flying. The threshold grows with the depth: max(min_jump, jump_ratio * z).
cv::Mat flying_pixels(const cv::Mat &depth, double jump_ratio, double min_jump) {
  cv::Mat mask(depth.size(), CV_8UC1, cv::Scalar(0));
  for (int v = 1; v + 1 < depth.rows; ++v) {
    const float *above = depth.ptr<float>(v - 1);
    const float *row = depth.ptr<float>(v);
    const float *below = depth.ptr<float>(v + 1);
    std::uint8_t *mask_row = mask.ptr<std::uint8_t>(v);
    for (int u = 1; u + 1 < depth.cols; ++u) {
      const float z = row[u];
      if (!std::isfinite(z)) {
        continue;
      }
      const double threshold = std::max(min_jump, jump_ratio * z);
      const bool horizontal = far_from(z, row[u - 1], threshold) && far_from(z, row[u + 1], threshold);
      const bool vertical = far_from(z, above[u], threshold) && far_from(z, below[u], threshold);
      if (horizontal || vertical) {
        mask_row[u] = 255;
      }
    }
  }
  return mask;
}

// Small obstacle clusters: connected groups of obstacle pixels (8-neighbourhood) whose real surface,
// seen by the camera, is smaller than min_area m². One pixel at depth z covers (z/fx) x (z/fy) m², so the
// area does not depend on the distance: a far rack upright has few pixels but a large area and is kept,
// a speck of noise near the camera has a tiny area and is removed. Removed pixels become unknown.
// Returns the number of removed clusters.
int remove_small_clusters(cv::Mat &labels, const cv::Mat &depth, const CameraModel &camera, double min_area) {
  const cv::Mat obstacle = labels == static_cast<std::uint8_t>(PixelClass::kObstacle);
  cv::Mat component;
  const int count = cv::connectedComponents(obstacle, component, 8, CV_32S);  // 0 is the background
  std::vector<double> area(static_cast<std::size_t>(count), 0.0);
  for (int v = 0; v < labels.rows; ++v) {
    const int *component_row = component.ptr<int>(v);
    const float *depth_row = depth.ptr<float>(v);
    for (int u = 0; u < labels.cols; ++u) {
      if (component_row[u] > 0) {
        const double z = depth_row[u];
        area[static_cast<std::size_t>(component_row[u])] += (z / camera.fx) * (z / camera.fy);
      }
    }
  }
  int removed = 0;
  for (int id = 1; id < count; ++id) {
    removed += area[static_cast<std::size_t>(id)] < min_area;
  }
  if (removed == 0) {
    return 0;
  }
  for (int v = 0; v < labels.rows; ++v) {
    const int *component_row = component.ptr<int>(v);
    std::uint8_t *label_row = labels.ptr<std::uint8_t>(v);
    for (int u = 0; u < labels.cols; ++u) {
      const int id = component_row[u];
      if (id > 0 && area[static_cast<std::size_t>(id)] < min_area) {
        label_row[u] = static_cast<std::uint8_t>(PixelClass::kUnknown);
      }
    }
  }
  return removed;
}

}  // namespace

// Parameters are read once, when the node starts. Defaults of the classification thresholds are those
// of the ground truth labels of method B (ml/perception/labels.py), so both methods are scored alike.
GeometricClassifier::GeometricClassifier(rclcpp::Node &node)
    : floor_z_(node.declare_parameter<double>("classical.floor_z", -0.095)),
      floor_tolerance_(node.declare_parameter<double>("classical.floor_tolerance", 0.02)),
      robot_height_(node.declare_parameter<double>("classical.robot_height", 1.0)),
      ransac_enabled_(node.declare_parameter<bool>("classical.ransac.enabled", true)),
      ransac_iterations_(static_cast<int>(node.declare_parameter<int>("classical.ransac.iterations", 100))),
      ransac_stride_(static_cast<int>(node.declare_parameter<int>("classical.ransac.stride", 4))),
      ransac_band_(node.declare_parameter<double>("classical.ransac.band", 0.25)),
      ransac_threshold_(node.declare_parameter<double>("classical.ransac.threshold", 0.02)),
      ransac_max_offset_(node.declare_parameter<double>("classical.ransac.max_offset", 0.08)),
      ransac_min_cos_tilt_(
          std::cos(node.declare_parameter<double>("classical.ransac.max_tilt_deg", 10.0) * kPi / 180.0)),
      ransac_min_inliers_(static_cast<int>(node.declare_parameter<int>("classical.ransac.min_inliers", 300))),
      flying_enabled_(node.declare_parameter<bool>("classical.flying_pixels.enabled", true)),
      flying_jump_ratio_(node.declare_parameter<double>("classical.flying_pixels.jump_ratio", 0.05)),
      flying_min_jump_(node.declare_parameter<double>("classical.flying_pixels.min_jump", 0.05)),
      clusters_enabled_(node.declare_parameter<bool>("classical.clusters.enabled", true)),
      clusters_min_area_(node.declare_parameter<double>("classical.clusters.min_area", 0.001)),
      logger_(node.get_logger()),
      clock_(node.get_clock()) {
  if (floor_tolerance_ <= 0.0) {
    throw std::invalid_argument("classical.floor_tolerance must be > 0");
  }
  if (robot_height_ <= floor_tolerance_) {
    throw std::invalid_argument("classical.robot_height must be > classical.floor_tolerance");
  }
  if (ransac_iterations_ < 1 || ransac_stride_ < 1 || ransac_min_inliers_ < 3) {
    throw std::invalid_argument("classical.ransac: iterations >= 1, stride >= 1, min_inliers >= 3");
  }
  if (ransac_band_ <= 0.0 || ransac_threshold_ <= 0.0 || ransac_max_offset_ <= 0.0) {
    throw std::invalid_argument("classical.ransac: band, threshold and max_offset must be > 0");
  }
  if (flying_jump_ratio_ <= 0.0 || flying_min_jump_ <= 0.0) {
    throw std::invalid_argument("classical.flying_pixels: jump_ratio and min_jump must be > 0");
  }
  if (clusters_min_area_ < 0.0) {
    throw std::invalid_argument("classical.clusters.min_area must be >= 0");
  }
  last_floor_.offset = -floor_z_;
  RCLCPP_INFO(logger_,
              "classical: floor_z %.3f m, floor_tolerance %.3f m, robot_height %.2f m, RANSAC %s, flying pixels %s, "
              "obstacle clusters < %.4f m2 %s",
              floor_z_, floor_tolerance_, robot_height_, ransac_enabled_ ? "on" : "off",
              flying_enabled_ ? "removed" : "kept", clusters_min_area_, clusters_enabled_ ? "removed" : "kept");
}

bool GeometricClassifier::plausible_floor(const FloorPlane &floor) const {
  if (floor.normal.z() < ransac_min_cos_tilt_) {
    return false;  // too steep: a wall, a box side, not the floor
  }
  const double z_under_robot = -floor.offset / floor.normal.z();  // the plane at x = y = 0
  return std::abs(z_under_robot - floor_z_) <= ransac_max_offset_;  // e.g. not the top of a 15 cm shelf
}

bool GeometricClassifier::find_floor(const std::vector<Eigen::Vector3d> &candidates, FloorPlane &floor) const {
  const int count = static_cast<int>(candidates.size());
  if (count < ransac_min_inliers_) {
    return false;
  }
  // Fixed seed: the same frame always gives the same floor, so experiments are reproducible.
  std::mt19937 random(0);
  std::uniform_int_distribution<int> pick(0, count - 1);

  FloorPlane best;
  int best_inliers = 0;
  for (int iteration = 0; iteration < ransac_iterations_; ++iteration) {
    FloorPlane hypothesis;
    if (!plane_from_points(candidates[pick(random)], candidates[pick(random)], candidates[pick(random)],
                           hypothesis) ||
        !plausible_floor(hypothesis)) {
      continue;
    }
    int inliers = 0;
    for (const auto &p : candidates) {
      if (std::abs(hypothesis.height(p)) <= ransac_threshold_) {
        ++inliers;
      }
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best = hypothesis;
    }
  }
  if (best_inliers < ransac_min_inliers_) {
    return false;
  }

  // Refine: least squares through the inliers of the best hypothesis (three points are noisy).
  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(static_cast<std::size_t>(best_inliers));
  for (const auto &p : candidates) {
    if (std::abs(best.height(p)) <= ransac_threshold_) {
      inliers.push_back(p);
    }
  }
  const FloorPlane refined = fit_plane(inliers);
  floor = plausible_floor(refined) ? refined : best;
  return true;
}

cv::Mat GeometricClassifier::classify(const RgbdFrame &frame) {
  const CameraModel &camera = frame.camera;
  // Pixel + depth -> point in camera_optical_frame (pinhole model) -> point in base_link (pose from TF).
  const auto to_base = [&](int u, int v, double z) {
    const Eigen::Vector3d p_camera((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z);
    return Eigen::Vector3d(frame.base_from_camera * p_camera);
  };

  // 0. Flying pixels at depth jumps: no reliable point -> unknown, and not a floor candidate.
  const cv::Mat flying = flying_enabled_ ? flying_pixels(frame.depth, flying_jump_ratio_, flying_min_jump_)
                                         : cv::Mat(frame.depth.size(), CV_8UC1, cv::Scalar(0));
  last_flying_count_ = cv::countNonZero(flying);

  // 1. Floor of this frame. Prior: horizontal plane z = floor_z_ (robot standing on a flat floor).
  FloorPlane floor;
  floor.offset = -floor_z_;
  bool found = false;
  if (ransac_enabled_) {
    std::vector<Eigen::Vector3d> candidates;
    for (int v = 0; v < frame.depth.rows; v += ransac_stride_) {
      const float *depth_row = frame.depth.ptr<float>(v);
      const std::uint8_t *flying_row = flying.ptr<std::uint8_t>(v);
      for (int u = 0; u < frame.depth.cols; u += ransac_stride_) {
        const double z = depth_row[u];
        if (!std::isfinite(z) || flying_row[u]) {
          continue;
        }
        const Eigen::Vector3d p = to_base(u, v, z);
        if (std::abs(p.z() - floor_z_) <= ransac_band_) {
          candidates.push_back(p);
        }
      }
    }
    found = find_floor(candidates, floor);
    if (!found) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                           "classical: floor not found (%zu candidates), using the fixed floor z = %.3f m",
                           candidates.size(), floor_z_);
    }
  }
  last_floor_ = floor;
  last_floor_found_ = found;

  // 2. Class of every pixel by its height over the floor (same rules as the ground truth).
  // Start with "unknown" everywhere: a pixel we skip (no depth) stays unknown, never free.
  cv::Mat labels(frame.depth.size(), CV_8UC1, cv::Scalar(static_cast<int>(PixelClass::kUnknown)));
  for (int v = 0; v < frame.depth.rows; ++v) {
    const float *depth_row = frame.depth.ptr<float>(v);
    const std::uint8_t *flying_row = flying.ptr<std::uint8_t>(v);
    std::uint8_t *label_row = labels.ptr<std::uint8_t>(v);
    for (int u = 0; u < frame.depth.cols; ++u) {
      const double z = depth_row[u];
      if (!std::isfinite(z) || flying_row[u]) {
        continue;  // NaN (no measurement) or a flying pixel -> stays unknown
      }
      const double height = floor.height(to_base(u, v, z));

      PixelClass label = PixelClass::kUnknown;
      if (std::abs(height) <= floor_tolerance_) {
        label = PixelClass::kFree;        // floor
      } else if (height < -floor_tolerance_) {
        label = PixelClass::kObstacle;    // below the floor: hole, edge or reflection -> never free
      } else if (height <= robot_height_) {
        label = PixelClass::kObstacle;    // between the floor and the top of the robot
      }                                   // else higher than the robot -> unknown
      label_row[u] = static_cast<std::uint8_t>(label);
    }
  }

  // 3. Small obstacle clusters (noise) -> unknown; thin but tall rack uprights have a large area and stay.
  last_removed_clusters_ = clusters_enabled_ ? remove_small_clusters(labels, frame.depth, camera, clusters_min_area_) : 0;
  return labels;
}

}  // namespace capstone_perception
