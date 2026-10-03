#include "capstone_perception/pixel_classifier.hpp"

#include <cmath>

namespace capstone_perception {

std::string validate_labels(const cv::Mat &labels, const RgbdFrame &frame) {
  if (labels.type() != CV_8UC1) {
    return "labels must be CV_8UC1";
  }
  if (labels.size() != frame.depth.size()) {
    return "labels must have the size of the depth image";
  }
  for (int v = 0; v < labels.rows; ++v) {
    const auto *label_row = labels.ptr<std::uint8_t>(v);
    const auto *depth_row = frame.depth.ptr<float>(v);
    for (int u = 0; u < labels.cols; ++u) {
      const auto label = static_cast<PixelClass>(label_row[u]);
      if (label != PixelClass::kFree && label != PixelClass::kObstacle && label != PixelClass::kUnknown) {
        return "labels must be 0 (free), 1 (obstacle) or 255 (unknown)";
      }
      if (std::isnan(depth_row[u]) && label != PixelClass::kUnknown) {
        return "a pixel without depth (NaN) must be unknown";
      }
    }
  }
  return "";
}

}  // namespace capstone_perception
