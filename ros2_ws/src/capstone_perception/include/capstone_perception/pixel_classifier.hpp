#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

#include "capstone_perception/rgbd_frame.hpp"

namespace capstone_perception {

// Class of one pixel in the label image returned by a PixelClassifier.
enum class PixelClass : std::uint8_t {
  kFree = 0,        // traversable floor
  kObstacle = 1,    // something the robot must not drive into
  kUnknown = 255,   // no reliable decision: no depth, outside the trusted range, uncertain
};

// The common interface of the two G2 methods. Both answer the same question — "what is this pixel?" —
// and GridBuilder turns the answer into /perception/obstacles in the same way for both, so the
// comparison of the methods is fair.
//
// Rules for implementations:
// - classify() returns a CV_8UC1 image of the size of frame.depth with values of PixelClass only;
// - a pixel whose depth is NaN must be kUnknown: free space is never declared without geometry;
// - throw std::runtime_error when the frame cannot be classified (for example, a broken model);
//   PerceptionNode then reports it and, for the learned method, falls back to the classical one.
class PixelClassifier {
 public:
  virtual ~PixelClassifier() = default;
  virtual std::string name() const = 0;
  virtual cv::Mat classify(const RgbdFrame &frame) = 0;
};

// Checks the contract above; returns an empty string when the labels are valid, otherwise the reason.
std::string validate_labels(const cv::Mat &labels, const RgbdFrame &frame);

}  // namespace capstone_perception
