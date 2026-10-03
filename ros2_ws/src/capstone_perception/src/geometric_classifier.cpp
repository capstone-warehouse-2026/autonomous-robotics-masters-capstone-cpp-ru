#include "capstone_perception/geometric_classifier.hpp"

namespace capstone_perception {

GeometricClassifier::GeometricClassifier(rclcpp::Node & /*node*/) {
  // Participant 1: declare the parameters of method A here, e.g.
  // node.declare_parameter<double>("classical.floor_tolerance", 0.03);
}

cv::Mat GeometricClassifier::classify(const RgbdFrame &frame) {
  // Participant 1, method A (projects/02_perception.md):
  //  1. remove flying pixels at depth discontinuities;
  //  2. back-project pixels to points and transform them with frame.base_from_camera into base_link;
  //  3. fit the floor plane by RANSAC near the expected floor (z = -0.095 m in base_link, normal up);
  //  4. height over the floor: near 0 -> kFree, between ~5 cm and the robot height -> kObstacle,
  //     above the robot -> kUnknown (the robot passes under it);
  //  5. drop tiny clusters of obstacle points without losing thin obstacles (rack uprights, 8 cm).
  // Until then every pixel is unknown, so the node runs end to end and publishes an all -1 grid.
  return cv::Mat(frame.depth.size(), CV_8UC1, cv::Scalar(static_cast<int>(PixelClass::kUnknown)));
}

}  // namespace capstone_perception
