#include "capstone_perception/cnn_classifier.hpp"

#include <stdexcept>

namespace capstone_perception {

CnnClassifier::CnnClassifier(rclcpp::Node &node) {
  // Participant 2: load the ONNX model here (ONNX Runtime is added to the dev image on request, issue `infra`)
  // and check its input/output shapes; a missing or broken model must not crash the node.
  model_path_ = node.declare_parameter<std::string>("learned.model_path", "");
}

cv::Mat CnnClassifier::classify(const RgbdFrame & /*frame*/) {
  // Participant 2, method B (projects/02_perception.md):
  //  1. preprocess rgb + depth exactly as in training (size, normalization, NaN handling);
  //  2. run the network, take the arg-max class per pixel and resize to the depth size;
  //  3. force kUnknown wherever the depth is NaN: colour alone never declares free space;
  //  4. throw std::runtime_error on non-finite network outputs.
  // Throwing makes PerceptionNode fall back to the classical method and report it in /diagnostics.
  if (model_path_.empty()) {
    throw std::runtime_error("learned.model_path is not set");
  }
  throw std::runtime_error("CNN inference is not implemented yet");
}

}  // namespace capstone_perception
