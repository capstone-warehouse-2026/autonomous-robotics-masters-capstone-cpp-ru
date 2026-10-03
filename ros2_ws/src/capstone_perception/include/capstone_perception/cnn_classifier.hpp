#pragma once

#include <string>

#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>

#include "capstone_perception/pixel_classifier.hpp"

namespace capstone_perception {

// Method B, learned (participant 2): a compact RGB-D CNN with classes free / obstacle / unknown, trained
// offline, run in C++ (ONNX Runtime), see projects/02_perception.md. Parameters are declared on the node
// under the "learned." prefix (for example learned.model_path).
class CnnClassifier : public PixelClassifier {
 public:
  explicit CnnClassifier(rclcpp::Node &node);
  std::string name() const override { return "learned"; }
  cv::Mat classify(const RgbdFrame &frame) override;

 private:
  std::string model_path_;
};

}  // namespace capstone_perception
