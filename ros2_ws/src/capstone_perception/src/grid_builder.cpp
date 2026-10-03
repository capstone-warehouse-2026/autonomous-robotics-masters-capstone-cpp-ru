#include "capstone_perception/grid_builder.hpp"

#include <cmath>
#include <stdexcept>

#include "capstone_perception/pixel_classifier.hpp"

namespace capstone_perception {

GridBuilder::GridBuilder(double resolution, double size, int pixel_stride, int min_free_points,
                         int min_obstacle_points)
    : resolution_(resolution),
      cells_(static_cast<int>(std::lround(size / resolution))),
      pixel_stride_(pixel_stride),
      min_free_points_(min_free_points),
      min_obstacle_points_(min_obstacle_points) {
  if (!(resolution > 0.0) || cells_ <= 0 || pixel_stride < 1 || min_free_points < 1 || min_obstacle_points < 1) {
    throw std::invalid_argument("GridBuilder: resolution, size, stride and vote thresholds must be positive");
  }
}

LocalGrid GridBuilder::build(const cv::Mat &labels, const RgbdFrame &frame,
                             const Eigen::Isometry3d &target_from_camera, double robot_x, double robot_y) const {
  LocalGrid grid;
  grid.resolution = resolution_;
  grid.width = cells_;
  grid.height = cells_;
  const double half = cells_ * resolution_ / 2.0;
  grid.origin_x = std::floor((robot_x - half) / resolution_) * resolution_;
  grid.origin_y = std::floor((robot_y - half) / resolution_) * resolution_;

  const std::size_t count = static_cast<std::size_t>(cells_) * static_cast<std::size_t>(cells_);
  std::vector<int> free_votes(count, 0);
  std::vector<int> obstacle_votes(count, 0);
  const CameraModel &k = frame.camera;
  for (int v = 0; v < labels.rows; v += pixel_stride_) {
    const auto *label_row = labels.ptr<std::uint8_t>(v);
    const auto *depth_row = frame.depth.ptr<float>(v);
    for (int u = 0; u < labels.cols; u += pixel_stride_) {
      const auto label = static_cast<PixelClass>(label_row[u]);
      const float z = depth_row[u];
      if (label == PixelClass::kUnknown || !std::isfinite(z) || z <= 0.0F) {
        continue;
      }
      const Eigen::Vector3d point = target_from_camera * Eigen::Vector3d((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
      const auto x = static_cast<long>(std::floor((point.x() - grid.origin_x) / resolution_));
      const auto y = static_cast<long>(std::floor((point.y() - grid.origin_y) / resolution_));
      if (x < 0 || y < 0 || x >= cells_ || y >= cells_) {
        continue;
      }
      const std::size_t cell = static_cast<std::size_t>(y) * cells_ + static_cast<std::size_t>(x);
      if (label == PixelClass::kObstacle) {
        ++obstacle_votes[cell];
      } else {
        ++free_votes[cell];
      }
    }
  }

  grid.data.assign(count, -1);
  for (std::size_t cell = 0; cell < count; ++cell) {
    if (obstacle_votes[cell] >= min_obstacle_points_) {
      grid.data[cell] = 100;
    } else if (free_votes[cell] >= min_free_points_) {
      grid.data[cell] = 0;
    }
  }
  return grid;
}

}  // namespace capstone_perception
