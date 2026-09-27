#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_sim_rgbd/rgbd_image.hpp"

namespace {

using capstone_sim_rgbd::bgra_to_rgb;
using capstone_sim_rgbd::Intrinsics;
using capstone_sim_rgbd::register_depth;
using capstone_sim_rgbd::webots_intrinsics;

constexpr float kInf = std::numeric_limits<float>::infinity();

TEST(RgbdImage, AstraIntrinsics) {
  const auto k = webots_intrinsics(640, 480, 1.04);
  EXPECT_NEAR(k.fx, 320.0 / std::tan(0.52), 1e-9);
  EXPECT_DOUBLE_EQ(k.fy, k.fx);
  EXPECT_DOUBLE_EQ(k.cx, 319.5);
  EXPECT_DOUBLE_EQ(k.cy, 239.5);
}

TEST(RgbdImage, BgraToRgbSwapsChannelsAndDropsAlpha) {
  const std::uint8_t bgra[] = {1, 2, 3, 255, 10, 20, 30, 0};
  std::uint8_t rgb[6] = {};
  bgra_to_rgb(bgra, 2, rgb);
  const std::uint8_t expected[] = {3, 2, 1, 30, 20, 10};
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(rgb[i], expected[i]);
  }
}

TEST(RgbdImage, ZeroOffsetKeepsDepthAndMarksMissingAsNan) {
  const auto k = webots_intrinsics(4, 1, 1.0);
  const std::vector<float> depth{1.0F, kInf, std::nanf(""), 2.5F};
  std::vector<float> out(4);
  register_depth(depth.data(), k, {0.0, 0.0, 0.0}, out.data());
  EXPECT_FLOAT_EQ(out[0], 1.0F);
  EXPECT_TRUE(std::isnan(out[1]));  // infinity: closer than minRange or farther than maxRange
  EXPECT_TRUE(std::isnan(out[2]));
  EXPECT_FLOAT_EQ(out[3], 2.5F);
}

TEST(RgbdImage, OffsetShiftsPlaneByDisparity) {
  // Flat wall at 2 m; the depth camera is 2.6 cm to the left of the color camera, as in Astra.
  const auto k = webots_intrinsics(640, 480, 1.04);
  std::vector<float> depth(640 * 480, 2.0F);
  std::vector<float> out(depth.size());
  register_depth(depth.data(), k, {-0.026, 0.0, 0.0}, out.data());
  const long shift = std::lround(k.fx * 0.026 / 2.0);  // pixels
  EXPECT_FLOAT_EQ(out[240 * 640 + 320 - shift], 2.0F);
  EXPECT_TRUE(std::isnan(out[240 * 640 + 639]));  // right edge: no depth point lands there
}

TEST(RgbdImage, NearestSurfaceWinsWhenPointsOverlap) {
  const Intrinsics k{1.0, 1.0, 1.5, 0.0, 4, 1};
  // Pixels 0 (1 m) and 1 (3 m) both land on pixel 2 after the offset.
  const std::vector<float> depth{1.0F, 3.0F, kInf, kInf};
  std::vector<float> out(4);
  register_depth(depth.data(), k, {1.5, 0.0, 0.0}, out.data());
  EXPECT_FLOAT_EQ(out[2], 1.0F);
}

}  // namespace
