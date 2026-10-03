#pragma once

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <rclcpp/time.hpp>

namespace capstone_perception {

// Pinhole model of the camera from /sensors/camera_info (no distortion in the simulator).
struct CameraModel {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;
};

// One synchronized RGB-D frame, the input of both methods (docs/INTERFACES.md, ADR 0003).
// Images are not copied from the ROS messages; a frame is valid only during one classify() call.
struct RgbdFrame {
  rclcpp::Time stamp;  // simulation time of the shot, the same for rgb, depth and camera_info
  cv::Mat rgb;         // CV_8UC3, RGB order
  cv::Mat depth;       // CV_32FC1, metres along the optical axis, registered to rgb; NaN = no measurement
  CameraModel camera;
  // Pose of camera_optical_frame (z forward, x right, y down) in base_link (x forward, z up).
  Eigen::Isometry3d base_from_camera = Eigen::Isometry3d::Identity();
};

}  // namespace capstone_perception
