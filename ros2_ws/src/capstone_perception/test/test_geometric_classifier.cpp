#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "capstone_perception/geometric_classifier.hpp"

namespace {

using capstone_perception::GeometricClassifier;
using capstone_perception::PixelClass;
using capstone_perception::RgbdFrame;
using capstone_perception::validate_labels;

constexpr std::uint8_t kFree = static_cast<std::uint8_t>(PixelClass::kFree);
constexpr std::uint8_t kObstacle = static_cast<std::uint8_t>(PixelClass::kObstacle);
constexpr std::uint8_t kUnknown = static_cast<std::uint8_t>(PixelClass::kUnknown);
constexpr double kPi = 3.14159265358979323846;
constexpr double kFloorZ = -0.095;  // floor of a robot standing on a flat floor, base_link

// Box in base_link (axis aligned), from the floor up to `top`.
struct Box {
  double x_min, x_max, y_min, y_max, top;
};

// Synthetic frame of the simulator camera (Orbbec Astra on TIAGo Base, ADR 0003), 640 x 480, the pose of camera_optical_frame in base_link as published by the simulator.
// The floor is tilted forward by `pitch_deg` in base_link (the robot pitching while braking).
// Depth is NaN outside 0.6..8 m, like the real camera.
RgbdFrame render(double pitch_deg, const std::vector<Box> &boxes = {}) {
  RgbdFrame frame;
  frame.camera = {558.89, 558.89, 319.5, 239.5, 640, 480};
  frame.rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  frame.depth = cv::Mat(480, 640, CV_32FC1);
  Eigen::Matrix3d rotation;
  rotation << 0.000, -0.389, 0.921,  //
      -1.000, 0.000, 0.000,          //
      0.000, -0.921, -0.389;
  frame.base_from_camera = Eigen::Isometry3d::Identity();
  frame.base_from_camera.linear() = rotation;
  frame.base_from_camera.translation() = Eigen::Vector3d(0.138, 0.011, 0.826);

  const double pitch = pitch_deg * kPi / 180.0;
  const Eigen::Vector3d floor_normal(std::sin(pitch), 0.0, std::cos(pitch));
  const double floor_offset = -floor_normal.dot(Eigen::Vector3d(0.0, 0.0, kFloorZ));
  const Eigen::Vector3d origin = frame.base_from_camera.translation();

  for (int v = 0; v < 480; ++v) {
    for (int u = 0; u < 640; ++u) {
      // Ray of the pixel; a point at depth t is origin + t * ray (t is the optical z, i.e. the depth).
      const Eigen::Vector3d ray_camera((u - frame.camera.cx) / frame.camera.fx, (v - frame.camera.cy) / frame.camera.fy,
                                       1.0);
      const Eigen::Vector3d ray = rotation * ray_camera;
      double t = std::numeric_limits<double>::infinity();
      const double towards_floor = floor_normal.dot(ray);
      if (towards_floor < 0.0) {
        t = -(floor_normal.dot(origin) + floor_offset) / towards_floor;
      }
      for (const auto &box : boxes) {  // slab test of a ray against an axis aligned box
        const Eigen::Vector3d low(box.x_min, box.y_min, kFloorZ), high(box.x_max, box.y_max, kFloorZ + box.top);
        double enter = 0.0, leave = std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis) {
          double t1 = (low[axis] - origin[axis]) / ray[axis], t2 = (high[axis] - origin[axis]) / ray[axis];
          if (t1 > t2) std::swap(t1, t2);
          enter = std::max(enter, t1);
          leave = std::min(leave, t2);
        }
        if (enter <= leave && enter < t) t = enter;
      }
      frame.depth.at<float>(v, u) =
          (t >= 0.6 && t <= 8.0) ? static_cast<float>(t) : std::numeric_limits<float>::quiet_NaN();
    }
  }
  return frame;
}

class GeometricClassifierTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static std::shared_ptr<rclcpp::Node> node(const std::vector<rclcpp::Parameter> &overrides = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    static int counter = 0;  // unique node names: several classifiers live at the same time in one test
    return std::make_shared<rclcpp::Node>("classical_test_" + std::to_string(counter++), options);
  }
};

// Share of the pixels with the given label among the pixels with depth.
double share(const cv::Mat &labels, const cv::Mat &depth, std::uint8_t label) {
  int valid = 0, matching = 0;
  for (int v = 0; v < labels.rows; ++v) {
    for (int u = 0; u < labels.cols; ++u) {
      if (std::isfinite(depth.at<float>(v, u))) {
        ++valid;
        matching += labels.at<std::uint8_t>(v, u) == label;
      }
    }
  }
  return valid ? static_cast<double>(matching) / valid : 0.0;
}

TEST_F(GeometricClassifierTest, FlatFloorIsFreeAndFound) {
  auto n = node();
  GeometricClassifier classifier(*n);
  const auto frame = render(0.0);
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_EQ(validate_labels(labels, frame), "");
  EXPECT_TRUE(classifier.last_floor_found());
  EXPECT_NEAR(classifier.last_floor().normal.z(), 1.0, 1e-3);
  EXPECT_NEAR(classifier.last_floor().offset, -kFloorZ, 2e-3);
  EXPECT_GT(share(labels, frame.depth, kFree), 0.99);
}

TEST_F(GeometricClassifierTest, MissingDepthIsUnknown) {
  auto n = node();
  GeometricClassifier classifier(*n);
  auto frame = render(0.0);
  frame.depth(cv::Rect(280, 360, 80, 80)).setTo(std::numeric_limits<float>::quiet_NaN());  // a hole in the floor view
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_EQ(validate_labels(labels, frame), "");
  EXPECT_EQ(labels.at<std::uint8_t>(400, 320), kUnknown);
}

TEST_F(GeometricClassifierTest, BoxOnTheFloorIsObstacle) {
  auto n = node();
  GeometricClassifier classifier(*n);
  const auto frame = render(0.0, {{2.0, 2.5, -0.3, 0.3, 0.30}});
  const cv::Mat labels = classifier.classify(frame);
  // The centre pixel looks at the floor 2.3 m ahead, inside the box footprint: it sees the box.
  EXPECT_EQ(labels.at<std::uint8_t>(240, 320), kObstacle);
  EXPECT_TRUE(classifier.last_floor_found());
  EXPECT_NEAR(classifier.last_floor().offset, -kFloorZ, 2e-3);
}

TEST_F(GeometricClassifierTest, ThinUprightIsObstacle) {
  auto n = node();
  GeometricClassifier classifier(*n);
  const auto frame = render(0.0, {{3.0, 3.08, -0.04, 0.04, 2.2}});  // 8 cm rack upright, 3 m ahead
  const cv::Mat labels = classifier.classify(frame);
  int obstacle = 0;
  for (int v = 0; v < labels.rows; ++v) {
    obstacle += labels.at<std::uint8_t>(v, 320) == kObstacle;
  }
  EXPECT_GT(obstacle, 20);
}

TEST_F(GeometricClassifierTest, ShelfTopIsNotTakenForTheFloor) {
  auto n = node();
  GeometricClassifier classifier(*n);
  // Wide low shelf (15 cm) right in front: a big horizontal plane 15 cm over the floor.
  const auto frame = render(0.0, {{1.5, 4.0, -2.0, 0.2, 0.15}});
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_NEAR(classifier.last_floor().offset, -kFloorZ, 0.02);
  EXPECT_EQ(labels.at<std::uint8_t>(240, 400), kObstacle);  // a pixel on the shelf top
}

TEST_F(GeometricClassifierTest, TiltedFloorIsFoundByRansac) {
  const auto frame = render(3.0);  // robot pitched by 3 degrees
  auto fixed_node = node({rclcpp::Parameter("classical.ransac.enabled", false)});
  GeometricClassifier fixed(*fixed_node);
  auto ransac_node = node();
  GeometricClassifier ransac(*ransac_node);

  const double free_fixed = share(fixed.classify(frame), frame.depth, kFree);
  const double free_ransac = share(ransac.classify(frame), frame.depth, kFree);
  EXPECT_TRUE(ransac.last_floor_found());
  EXPECT_NEAR(std::acos(ransac.last_floor().normal.z()) * 180.0 / kPi, 3.0, 0.2);
  EXPECT_GT(free_ransac, 0.99);
  EXPECT_LT(free_fixed, 0.5);  // without RANSAC most of the floor is taken for an obstacle or a hole
}

TEST_F(GeometricClassifierTest, FallsBackToFixedFloorWithoutCandidates) {
  auto n = node();
  GeometricClassifier classifier(*n);
  auto frame = render(0.0);
  frame.depth.setTo(std::numeric_limits<float>::quiet_NaN());
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_FALSE(classifier.last_floor_found());
  EXPECT_EQ(validate_labels(labels, frame), "");
  EXPECT_EQ(cv::countNonZero(labels != kUnknown), 0);
}

TEST_F(GeometricClassifierTest, FloorHasNoFlyingPixels) {
  auto n = node();
  GeometricClassifier classifier(*n);
  classifier.classify(render(0.0));
  EXPECT_EQ(classifier.last_flying_count(), 0);  // the floor up to 8 m changes smoothly from row to row
  classifier.classify(render(3.0));
  EXPECT_EQ(classifier.last_flying_count(), 0);
}

TEST_F(GeometricClassifierTest, MixedPixelsAtUprightBorderAreRemoved) {
  // Upright 8 cm wide, 3 m ahead; at its right border put a mixed pixel: the mean of the upright depth
  // and of the floor behind it, as a real camera does. Its point hangs in the air behind the upright.
  auto frame = render(0.0, {{3.0, 3.08, -0.04, 0.04, 2.2}});
  std::vector<cv::Point> mixed;
  for (int v = 0; v < frame.depth.rows; ++v) {
    for (int u = 320; u + 2 < frame.depth.cols; ++u) {
      const float front = frame.depth.at<float>(v, u), back = frame.depth.at<float>(v, u + 2);
      if (std::isfinite(front) && std::isfinite(back) && back - front > 0.5F &&
          frame.depth.at<float>(v, u + 1) - front > 0.5F) {
        frame.depth.at<float>(v, u + 1) = 0.5F * (front + back);
        mixed.emplace_back(u + 1, v);
        break;
      }
    }
  }
  ASSERT_GT(mixed.size(), 20U);

  auto keep_node = node({rclcpp::Parameter("classical.flying_pixels.enabled", false)});
  GeometricClassifier keep(*keep_node);
  const cv::Mat kept = keep.classify(frame);
  auto remove_node = node();
  GeometricClassifier remove(*remove_node);
  const cv::Mat removed = remove.classify(frame);

  int false_obstacles = 0, unknown = 0;
  for (const auto &p : mixed) {
    false_obstacles += kept.at<std::uint8_t>(p) == kObstacle;
    unknown += removed.at<std::uint8_t>(p) == kUnknown;
  }
  EXPECT_GT(false_obstacles, static_cast<int>(mixed.size()) / 2);  // without the filter: obstacles in the air
  EXPECT_EQ(unknown, static_cast<int>(mixed.size()));             // with the filter: all unknown
  EXPECT_EQ(validate_labels(removed, frame), "");
  int upright = 0;  // the upright itself is kept
  for (int v = 0; v < removed.rows; ++v) {
    upright += removed.at<std::uint8_t>(v, 320) == kObstacle;
  }
  EXPECT_GT(upright, 20);
}

TEST_F(GeometricClassifierTest, IsolatedSpeckleIsRemoved) {
  auto n = node();
  GeometricClassifier classifier(*n);
  auto frame = render(0.0);
  frame.depth.at<float>(400, 320) *= 0.5F;  // one wrong measurement on the floor: a "point in the air"
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_EQ(classifier.last_flying_count(), 1);
  EXPECT_EQ(labels.at<std::uint8_t>(400, 320), kUnknown);
  EXPECT_EQ(labels.at<std::uint8_t>(400, 321), kFree);
}

// Noise bumps on the floor: 2 x 2 pixel patches measured 5% too close, so their points float ~5 cm over
// the floor (obstacle by height, not flying: inside a patch every pixel has a neighbour of the same depth).
std::vector<cv::Point> add_noise_bumps(RgbdFrame &frame) {
  std::vector<cv::Point> bumps;
  for (int v = 300; v < 460; v += 40) {
    for (int u = 100; u < 560; u += 80) {
      cv::Mat patch = frame.depth(cv::Rect(u, v, 2, 2));
      patch *= 0.95;
      bumps.emplace_back(u, v);
    }
  }
  return bumps;
}

TEST_F(GeometricClassifierTest, SmallNoiseClustersAreRemoved) {
  auto frame = render(0.0);
  const auto bumps = add_noise_bumps(frame);

  auto keep_node = node({rclcpp::Parameter("classical.clusters.enabled", false)});
  GeometricClassifier keep(*keep_node);
  const cv::Mat kept = keep.classify(frame);
  auto remove_node = node();
  GeometricClassifier remove(*remove_node);
  const cv::Mat removed = remove.classify(frame);

  int noise_obstacles = 0, noise_unknown = 0;
  for (const auto &p : bumps) {
    noise_obstacles += kept.at<std::uint8_t>(p) == kObstacle;
    noise_unknown += removed.at<std::uint8_t>(p) == kUnknown;
  }
  EXPECT_EQ(noise_obstacles, static_cast<int>(bumps.size()));  // without the filter: false obstacles
  EXPECT_EQ(noise_unknown, static_cast<int>(bumps.size()));    // with the filter: removed
  EXPECT_EQ(remove.last_removed_clusters(), static_cast<int>(bumps.size()));
  EXPECT_EQ(validate_labels(removed, frame), "");
}

TEST_F(GeometricClassifierTest, SmallRealBoxIsKept) {
  auto n = node();
  GeometricClassifier classifier(*n);
  const auto frame = render(0.0, {{2.3, 2.4, -0.05, 0.05, 0.10}});  // 10 x 10 x 10 cm box, 2.3 m ahead
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_EQ(classifier.last_removed_clusters(), 0);
  int obstacle = 0;
  for (int v = 0; v < labels.rows; ++v) {
    obstacle += labels.at<std::uint8_t>(v, 320) == kObstacle;
  }
  EXPECT_GT(obstacle, 10);
}

TEST_F(GeometricClassifierTest, FarThinUprightIsKept) {
  auto n = node();
  GeometricClassifier classifier(*n);
  const auto frame = render(0.0, {{6.0, 6.08, -0.04, 0.04, 2.2}});  // 8 cm upright, 6 m ahead: ~7 px wide
  const cv::Mat labels = classifier.classify(frame);
  EXPECT_EQ(classifier.last_removed_clusters(), 0);
  int obstacle = 0;
  for (int v = 0; v < labels.rows; ++v) {
    obstacle += labels.at<std::uint8_t>(v, 320) == kObstacle;
  }
  EXPECT_GT(obstacle, 20);
}

TEST_F(GeometricClassifierTest, RejectsInvalidParameters) {
  auto bad_clusters = node({rclcpp::Parameter("classical.clusters.min_area", -1.0)});
  EXPECT_THROW(GeometricClassifier classifier(*bad_clusters), std::invalid_argument);
  auto bad_flying = node({rclcpp::Parameter("classical.flying_pixels.jump_ratio", 0.0)});
  EXPECT_THROW(GeometricClassifier classifier(*bad_flying), std::invalid_argument);
  auto bad_tolerance = node({rclcpp::Parameter("classical.floor_tolerance", 0.0)});
  EXPECT_THROW(GeometricClassifier classifier(*bad_tolerance), std::invalid_argument);
  auto bad_height = node({rclcpp::Parameter("classical.robot_height", 0.01)});
  EXPECT_THROW(GeometricClassifier classifier(*bad_height), std::invalid_argument);
  auto bad_ransac = node({rclcpp::Parameter("classical.ransac.stride", 0)});
  EXPECT_THROW(GeometricClassifier classifier(*bad_ransac), std::invalid_argument);
}

}  // namespace
