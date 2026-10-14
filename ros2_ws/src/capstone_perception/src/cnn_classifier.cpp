#include "capstone_perception/cnn_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace capstone_perception {

namespace {

constexpr int kInputChannels = 5;
constexpr int kClasses = 3;
constexpr float kMaxDepth = 8.0f;  // m, preprocess.py MAX_DEPTH

// One ONNX Runtime environment per process, shared by all sessions.
Ort::Env &environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "capstone_perception");
  return env;
}

std::string shape_text(const std::vector<std::int64_t> &shape) {
  std::string text;
  for (const auto dimension : shape) {
    text += (text.empty() ? "" : "x") + std::to_string(dimension);
  }
  return text;
}

}  // namespace

void network_input(const RgbdFrame &frame, int height, int width, float *tensor) {
  const std::size_t plane = static_cast<std::size_t>(height) * width;
  for (int r = 0; r < height; ++r) {
    const auto *rgb_top = frame.rgb.ptr<cv::Vec3b>(2 * r);
    const auto *rgb_bottom = frame.rgb.ptr<cv::Vec3b>(2 * r + 1);
    const auto *depth_row = frame.depth.ptr<float>(2 * r);
    for (int c = 0; c < width; ++c) {
      const std::size_t index = static_cast<std::size_t>(r) * width + c;
      for (int k = 0; k < 3; ++k) {
        // Integer sums are exact in float, so the result equals numpy's float32 mean / 255.
        const float sum = static_cast<float>(rgb_top[2 * c][k]) + rgb_top[2 * c + 1][k] + rgb_bottom[2 * c][k] +
                          rgb_bottom[2 * c + 1][k];
        tensor[k * plane + index] = sum / 4.0f / 255.0f;
      }
      const float depth = depth_row[2 * c];
      const bool valid = std::isfinite(depth);
      tensor[3 * plane + index] = valid ? std::clamp(depth, 0.0f, kMaxDepth) / kMaxDepth : 0.0f;
      tensor[4 * plane + index] = valid ? 1.0f : 0.0f;
    }
  }
}

cv::Mat labels_from_logits(const float *logits, int height, int width, const cv::Mat &depth) {
  const std::size_t plane = static_cast<std::size_t>(height) * width;
  cv::Mat labels(depth.size(), CV_8UC1);
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      const std::size_t index = static_cast<std::size_t>(r) * width + c;
      const float free = logits[index];
      const float obstacle = logits[plane + index];
      const float unknown = logits[2 * plane + index];
      if (!std::isfinite(free) || !std::isfinite(obstacle) || !std::isfinite(unknown)) {
        throw std::runtime_error("the network returned non-finite logits");
      }
      PixelClass label = PixelClass::kFree;
      float best = free;
      if (obstacle > best) {
        label = PixelClass::kObstacle;
        best = obstacle;
      }
      if (unknown > best) {
        label = PixelClass::kUnknown;
      }
      for (int v = 2 * r; v < 2 * r + 2; ++v) {
        for (int u = 2 * c; u < 2 * c + 2; ++u) {
          // Free space is never declared without geometry, whatever the network says.
          labels.at<std::uint8_t>(v, u) =
              static_cast<std::uint8_t>(std::isfinite(depth.at<float>(v, u)) ? label : PixelClass::kUnknown);
        }
      }
    }
  }
  return labels;
}

CnnModel::CnnModel(const std::string &path, int threads) {
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(threads);
  options.SetInterOpNumThreads(1);
  try {
    session_ = Ort::Session(environment(), path.c_str(), options);
  } catch (const Ort::Exception &error) {
    throw std::runtime_error("cannot load the model " + path + ": " + error.what());
  }
  if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
    throw std::runtime_error(path + ": the model must have one input and one output");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_.GetInputNameAllocated(0, allocator).get();
  output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
  const Ort::TypeInfo input_type = session_.GetInputTypeInfo(0);
  const Ort::TypeInfo output_type = session_.GetOutputTypeInfo(0);
  const auto input_info = input_type.GetTensorTypeAndShapeInfo();
  const auto output_info = output_type.GetTensorTypeAndShapeInfo();
  const std::vector<std::int64_t> input_shape = input_info.GetShape();
  const std::vector<std::int64_t> output_shape = output_info.GetShape();
  const bool shapes_valid = input_shape.size() == 4 && input_shape[0] == 1 && input_shape[1] == kInputChannels &&
                            input_shape[2] > 0 && input_shape[3] > 0 && output_shape.size() == 4 &&
                            output_shape[0] == 1 && output_shape[1] == kClasses &&
                            output_shape[2] == input_shape[2] && output_shape[3] == input_shape[3];
  if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || !shapes_valid) {
    throw std::runtime_error(path + ": expected float input 1x5xHxW and output 1x3xHxW, got " +
                             shape_text(input_shape) + " -> " + shape_text(output_shape));
  }
  height_ = static_cast<int>(input_shape[2]);
  width_ = static_cast<int>(input_shape[3]);

  input_.assign(static_cast<std::size_t>(kInputChannels) * height_ * width_, 0.0f);
  logits_.assign(static_cast<std::size_t>(kClasses) * height_ * width_, 0.0f);
  const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  input_tensor_ = Ort::Value::CreateTensor<float>(memory, input_.data(), input_.size(), input_shape.data(),
                                                  input_shape.size());
  logits_tensor_ = Ort::Value::CreateTensor<float>(memory, logits_.data(), logits_.size(), output_shape.data(),
                                                   output_shape.size());
}

const float *CnnModel::run() {
  const char *input_names[] = {input_name_.c_str()};
  const char *output_names[] = {output_name_.c_str()};
  session_.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &logits_tensor_, 1);
  return logits_.data();
}

CnnClassifier::CnnClassifier(rclcpp::Node &node) {
  std::string path = node.declare_parameter<std::string>("learned.model_path", "");
  const int threads = static_cast<int>(node.declare_parameter<int>("learned.threads", 2));
  try {
    if (path.empty()) {
      path = ament_index_cpp::get_package_share_directory("capstone_perception") + "/models/perception_cnn.onnx";
    }
    model_ = std::make_unique<CnnModel>(path, threads);
    RCLCPP_INFO(node.get_logger(), "learned: model %s, network input 5x%dx%d", path.c_str(), model_->height(),
                model_->width());
  } catch (const std::exception &error) {
    // The node keeps running: every frame falls back to the classical method and /diagnostics says why.
    load_error_ = error.what();
    RCLCPP_ERROR(node.get_logger(), "learned: %s", load_error_.c_str());
  }
}

cv::Mat CnnClassifier::classify(const RgbdFrame &frame) {
  if (!model_) {
    throw std::runtime_error(load_error_);
  }
  const int height = model_->height();
  const int width = model_->width();
  if (frame.rgb.type() != CV_8UC3 || frame.depth.type() != CV_32FC1 || frame.rgb.size() != frame.depth.size() ||
      frame.depth.rows != 2 * height || frame.depth.cols != 2 * width) {
    throw std::runtime_error("the model expects rgb8 and 32FC1 frames of " + std::to_string(2 * width) + "x" +
                             std::to_string(2 * height));
  }
  network_input(frame, height, width, model_->input());
  return labels_from_logits(model_->run(), height, width, frame.depth);
}

}  // namespace capstone_perception
