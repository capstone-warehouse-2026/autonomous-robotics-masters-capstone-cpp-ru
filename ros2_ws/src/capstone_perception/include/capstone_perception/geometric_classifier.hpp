#pragma once

#include <string>

#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>

#include "capstone_perception/pixel_classifier.hpp"

namespace capstone_perception {

// Method A, classical (participant 1): floor plane by RANSAC in base_link, height of every point over
// the floor, obstacle clusters, see projects/02_perception.md. Parameters are declared on the node
// under the "classical." prefix.
class GeometricClassifier : public PixelClassifier {
 public:
  explicit GeometricClassifier(rclcpp::Node &node);
  std::string name() const override { return "classical"; }
  cv::Mat classify(const RgbdFrame &frame) override;
};

}  // namespace capstone_perception
