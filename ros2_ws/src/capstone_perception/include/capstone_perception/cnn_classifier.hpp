#pragma once

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>

#include "capstone_perception/pixel_classifier.hpp"

namespace capstone_perception {

// Network input of method B for a frame of 2*height x 2*width pixels; a literal copy of
// ml/perception/preprocess.py network_input(). `tensor` holds 5 x height x width floats (channels, rows,
// columns): R, G, B / 255 as the mean of each 2x2 block, depth / 8 m clipped to [0, 1] from the top-left
// pixel of the block (0 where not finite), and the validity of that depth pixel (1 or 0).
void network_input(const RgbdFrame &frame, int height, int width, float *tensor);

// Pixel classes from the network logits (3 x height x width: free, obstacle, unknown) at the size of `depth`
// (2*height x 2*width); a literal copy of preprocess.py labels_from_logits(): the arg-max class (the first
// on ties), each output pixel copied to its 2x2 block, kUnknown wherever the depth is not finite.
// Throws std::runtime_error on non-finite logits.
cv::Mat labels_from_logits(const float *logits, int height, int width, const cv::Mat &depth);

// The ONNX model of method B in ONNX Runtime (CPU). The input and output buffers are allocated once:
// fill input(), call run(), read the logits. Throws std::runtime_error when the file cannot be loaded or
// its input is not float 1x5xHxW and its output float 1x3xHxW.
class CnnModel {
 public:
  CnnModel(const std::string &path, int threads);
  int height() const { return height_; }
  int width() const { return width_; }
  float *input() { return input_.data(); }
  const float *run();

 private:
  int height_ = 0;
  int width_ = 0;
  std::string input_name_;
  std::string output_name_;
  std::vector<float> input_;
  std::vector<float> logits_;
  Ort::Session session_{nullptr};
  Ort::Value input_tensor_{nullptr};
  Ort::Value logits_tensor_{nullptr};
};

// Method B, learned (participant 2): a compact RGB-D U-Net with classes free / obstacle / unknown, trained
// offline (ml/perception), run in C++ with ONNX Runtime, see projects/02_perception.md. Parameters on the
// node: learned.model_path (empty = models/perception_cnn.onnx of this package) and learned.threads.
// A missing or broken model does not stop the node: classify() throws, and the node falls back to the
// classical method and reports it in /diagnostics.
class CnnClassifier : public PixelClassifier {
 public:
  explicit CnnClassifier(rclcpp::Node &node);
  std::string name() const override { return "learned"; }
  cv::Mat classify(const RgbdFrame &frame) override;

 private:
  std::unique_ptr<CnnModel> model_;
  std::string load_error_;
};

}  // namespace capstone_perception
