#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include "capstone_perception/rgbd_frame.hpp"

namespace capstone_perception {

// Local window of /perception/obstacles: row-major cells (index y * width + x), 0 free, 100 obstacle,
// -1 unknown; the origin is the corner of cell (0, 0) in the grid frame (odom), docs/INTERFACES.md.
struct LocalGrid {
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.0;
  int width = 0;
  int height = 0;
  std::vector<std::int8_t> data;
};

// Projects the label image of either method into the local window around the robot, the same way for
// both methods. Every labelled pixel with a finite depth becomes a 3D point; the cell it falls into
// collects a free or an obstacle vote. A cell is an obstacle with at least min_obstacle_points obstacle
// votes, otherwise free with at least min_free_points free votes, otherwise unknown. The grid is not
// inflated: G3/G4 add the robot footprint.
class GridBuilder {
 public:
  GridBuilder(double resolution, double size, int pixel_stride, int min_free_points, int min_obstacle_points);

  // target_from_camera: camera_optical_frame in the grid frame; robot_x/robot_y: base_link origin in it.
  // The window is centred on the robot and its origin is a multiple of the resolution, so cells of
  // consecutive grids coincide.
  LocalGrid build(const cv::Mat &labels, const RgbdFrame &frame, const Eigen::Isometry3d &target_from_camera,
                  double robot_x, double robot_y) const;

 private:
  double resolution_;
  int cells_;
  int pixel_stride_;
  int min_free_points_;
  int min_obstacle_points_;
};

}  // namespace capstone_perception
