#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "capstone_perception/grid_builder.hpp"
#include "capstone_perception/pixel_classifier.hpp"

namespace {

using capstone_perception::GridBuilder;
using capstone_perception::PixelClass;
using capstone_perception::RgbdFrame;
using capstone_perception::validate_labels;

constexpr std::uint8_t kFree = static_cast<std::uint8_t>(PixelClass::kFree);
constexpr std::uint8_t kObstacle = static_cast<std::uint8_t>(PixelClass::kObstacle);
constexpr std::uint8_t kUnknown = static_cast<std::uint8_t>(PixelClass::kUnknown);

// 5x5 camera with a narrow field of view (neighbouring pixels 2 mm apart at 2 m), 1 m above the floor,
// looking along +x of the grid frame. The centre pixel at depth 2.025 m sees (2.025, 0.025, 1.0),
// the middle of cell (140, 100) of a 200x200 grid centred on the origin.
RgbdFrame make_frame(float depth) {
  RgbdFrame frame;
  frame.camera = {1000.0, 1000.0, 2.0, 2.0, 5, 5};
  frame.depth = cv::Mat(5, 5, CV_32FC1, cv::Scalar(depth));
  frame.rgb = cv::Mat(5, 5, CV_8UC3, cv::Scalar(0, 0, 0));
  return frame;
}

Eigen::Isometry3d target_from_camera() {
  Eigen::Matrix3d rotation;
  rotation << 0, 0, 1,   // optical z (forward) -> x
      -1, 0, 0,          // optical x (right) -> -y
      0, -1, 0;          // optical y (down) -> -z
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = Eigen::Vector3d(0.0, 0.025, 1.0);
  return pose;
}

int count(const std::vector<std::int8_t> &data, std::int8_t value) {
  return static_cast<int>(std::count(data.begin(), data.end(), value));
}

constexpr std::size_t kCentreCell = 100 * 200 + 140;

TEST(GridBuilder, ObstaclePixelLandsInExpectedCell) {
  const auto frame = make_frame(2.025F);
  cv::Mat labels(5, 5, CV_8UC1, cv::Scalar(kUnknown));
  labels.at<std::uint8_t>(2, 2) = kObstacle;
  const auto grid = GridBuilder(0.05, 10.0, 1, 1, 1).build(labels, frame, target_from_camera(), 0.0, 0.0);
  ASSERT_EQ(grid.width, 200);
  ASSERT_EQ(grid.data.size(), 200U * 200U);
  EXPECT_EQ(grid.data[kCentreCell], 100);
  EXPECT_EQ(count(grid.data, 100), 1);
  EXPECT_EQ(count(grid.data, -1), 200 * 200 - 1);
}

TEST(GridBuilder, UnknownLabelsAndMissingDepthGiveNoVotes) {
  auto frame = make_frame(std::numeric_limits<float>::quiet_NaN());
  frame.depth.at<float>(2, 2) = 2.025F;
  cv::Mat labels(5, 5, CV_8UC1, cv::Scalar(kFree));
  const auto grid = GridBuilder(0.05, 10.0, 1, 1, 1).build(labels, frame, target_from_camera(), 0.0, 0.0);
  EXPECT_EQ(grid.data[kCentreCell], 0);
  EXPECT_EQ(count(grid.data, 0), 1);
  EXPECT_EQ(count(grid.data, 100), 0);
}

TEST(GridBuilder, ObstacleVoteWinsOverFreeVotesInTheSameCell) {
  const auto frame = make_frame(2.025F);
  cv::Mat labels(5, 5, CV_8UC1, cv::Scalar(kFree));
  labels.at<std::uint8_t>(2, 2) = kObstacle;
  const auto grid = GridBuilder(0.05, 10.0, 1, 1, 1).build(labels, frame, target_from_camera(), 0.0, 0.0);
  EXPECT_EQ(grid.data[kCentreCell], 100);
}

TEST(GridBuilder, ObstacleNeedsMinObstacleVotes) {
  const auto frame = make_frame(2.025F);
  cv::Mat labels(5, 5, CV_8UC1, cv::Scalar(kFree));
  labels.at<std::uint8_t>(2, 2) = kObstacle;
  const auto grid = GridBuilder(0.05, 10.0, 1, 1, 2).build(labels, frame, target_from_camera(), 0.0, 0.0);
  EXPECT_EQ(grid.data[kCentreCell], 0);  // one obstacle vote is not enough, the free votes win
}

TEST(GridBuilder, WindowIsCentredOnRobotAndAlignedToCells) {
  const auto frame = make_frame(2.025F);
  const cv::Mat labels(5, 5, CV_8UC1, cv::Scalar(kUnknown));
  const auto grid = GridBuilder(0.05, 10.0, 1, 1, 1).build(labels, frame, target_from_camera(), 1.234, -3.21);
  EXPECT_NEAR(std::remainder(grid.origin_x, 0.05), 0.0, 1e-9);
  EXPECT_NEAR(std::remainder(grid.origin_y, 0.05), 0.0, 1e-9);
  EXPECT_NEAR(grid.origin_x + 5.0, 1.234, 0.05);
  EXPECT_NEAR(grid.origin_y + 5.0, -3.21, 0.05);
}

TEST(GridBuilder, RejectsNonPositiveParameters) {
  EXPECT_THROW(GridBuilder(0.0, 10.0, 1, 1, 1), std::invalid_argument);
  EXPECT_THROW(GridBuilder(0.05, 10.0, 0, 1, 1), std::invalid_argument);
}

TEST(ValidateLabels, AcceptsValidAndRejectsBrokenLabels) {
  auto frame = make_frame(2.0F);
  frame.depth.at<float>(0, 0) = std::numeric_limits<float>::quiet_NaN();
  cv::Mat labels(5, 5, CV_8UC1, cv::Scalar(kFree));
  labels.at<std::uint8_t>(0, 0) = kUnknown;
  EXPECT_EQ(validate_labels(labels, frame), "");

  labels.at<std::uint8_t>(0, 0) = kFree;  // free space without depth
  EXPECT_NE(validate_labels(labels, frame), "");
  labels.at<std::uint8_t>(0, 0) = 7;  // not a class
  EXPECT_NE(validate_labels(labels, frame), "");
  EXPECT_NE(validate_labels(cv::Mat(5, 5, CV_8UC3), frame), "");
  EXPECT_NE(validate_labels(cv::Mat(4, 5, CV_8UC1, cv::Scalar(kUnknown)), frame), "");
}

}  // namespace
