#include "capstone_perception/geometric_classifier.hpp"

#include <cmath>
#include <cstdint>

#include <Eigen/Core>

namespace capstone_perception {

namespace {

// v1: fixed thresholds, the same as the ground truth labels of method B (ml/perception/labels.py).
// They become ROS parameters "classical.*" in v2.
constexpr double kFloorZ = -0.095;         // m, floor height in base_link (robot standing on a flat floor)
constexpr double kFloorTolerance = 0.02;   // m, |height| <= this -> floor
constexpr double kRobotHeight = 1.0;       // m, higher than this -> the robot passes under it

}  // namespace

GeometricClassifier::GeometricClassifier(rclcpp::Node & /*node*/) {
  // v2: declare the "classical.*" parameters here.
}

cv::Mat GeometricClassifier::classify(const RgbdFrame &frame) {
  const CameraModel &camera = frame.camera;

  // Start with "unknown" everywhere: a pixel we skip (no depth) stays unknown, never free.
  cv::Mat labels(frame.depth.size(), CV_8UC1, cv::Scalar(static_cast<int>(PixelClass::kUnknown)));

  for (int v = 0; v < frame.depth.rows; ++v) {
    const float *depth_row = frame.depth.ptr<float>(v);
    std::uint8_t *label_row = labels.ptr<std::uint8_t>(v);

    for (int u = 0; u < frame.depth.cols; ++u) {
      const double z = depth_row[u];
      if (!std::isfinite(z)) {
        continue;  // NaN (no measurement) -> stays unknown
      }

      // 1. Pixel + depth -> point in camera_optical_frame (pinhole model).
      const Eigen::Vector3d p_camera((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z);

      // 2. Camera -> robot (base_link): one multiplication by the pose from TF.
      const Eigen::Vector3d p_base = frame.base_from_camera * p_camera;

      // 3. Height over the floor.
      const double height = p_base.z() - kFloorZ;

      // 4. Class by height (same rules as the ground truth).
      PixelClass label = PixelClass::kUnknown;
      if (std::abs(height) <= kFloorTolerance) {
        label = PixelClass::kFree;        // floor
      } else if (height < -kFloorTolerance) {
        label = PixelClass::kObstacle;    // below the floor: hole, edge or reflection -> never free
      } else if (height <= kRobotHeight) {
        label = PixelClass::kObstacle;    // between the floor and the top of the robot
      }                                   // else higher than the robot -> unknown
      label_row[u] = static_cast<std::uint8_t>(label);
    }
  }
  return labels;
}

}  // namespace capstone_perception
