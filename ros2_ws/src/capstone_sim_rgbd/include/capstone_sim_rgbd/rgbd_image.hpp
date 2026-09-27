#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace capstone_sim_rgbd {

struct Intrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;
};

// Pinhole model of a Webots camera: horizontal field of view, square pixels, principal point in
// the image center in ROS pixel coordinates (the center of the first pixel is 0, so cx = (w - 1) / 2).
inline Intrinsics webots_intrinsics(int width, int height, double horizontal_fov) {
  const double f = width / 2.0 / std::tan(horizontal_fov / 2.0);
  return {f, f, (width - 1) / 2.0, (height - 1) / 2.0, width, height};
}

// Webots camera images are BGRA; the contract requires rgb8.
inline void bgra_to_rgb(const std::uint8_t *bgra, std::size_t pixels, std::uint8_t *rgb) {
  for (std::size_t i = 0; i < pixels; ++i) {
    rgb[3 * i] = bgra[4 * i + 2];
    rgb[3 * i + 1] = bgra[4 * i + 1];
    rgb[3 * i + 2] = bgra[4 * i];
  }
}

// Re-projects a planar depth image (m) of the depth camera into the color camera, which has the same
// intrinsics and orientation. offset is the depth camera origin in the color optical frame (m).
// Webots returns infinity both below minRange and above maxRange, so every pixel without a measurement,
// including pixels no depth point lands on, becomes NaN ("invalid", REP 117): an object that is too close
// must never look like free space.
inline void register_depth(const float *depth, const Intrinsics &k, const std::array<double, 3> &offset,
                           float *registered) {
  const std::size_t n = static_cast<std::size_t>(k.width) * static_cast<std::size_t>(k.height);
  for (std::size_t i = 0; i < n; ++i) {
    registered[i] = std::numeric_limits<float>::quiet_NaN();
  }
  for (int v = 0; v < k.height; ++v) {
    for (int u = 0; u < k.width; ++u) {
      const float z = depth[static_cast<std::size_t>(v) * k.width + u];
      if (!std::isfinite(z) || z <= 0.0F) {
        continue;
      }
      const double x = (u - k.cx) * z / k.fx + offset[0];
      const double y = (v - k.cy) * z / k.fy + offset[1];
      const double zc = z + offset[2];
      if (zc <= 0.0) {
        continue;
      }
      const long u2 = std::lround(k.fx * x / zc + k.cx);
      const long v2 = std::lround(k.fy * y / zc + k.cy);
      if (u2 < 0 || v2 < 0 || u2 >= k.width || v2 >= k.height) {
        continue;
      }
      float &target = registered[static_cast<std::size_t>(v2) * k.width + static_cast<std::size_t>(u2)];
      if (std::isnan(target) || zc < target) {
        target = static_cast<float>(zc);  // nearest surface wins where points overlap
      }
    }
  }
}

}  // namespace capstone_sim_rgbd
