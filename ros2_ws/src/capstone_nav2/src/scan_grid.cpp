#include "capstone_nav2/scan_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capstone_nav2 {

nav_msgs::msg::OccupancyGrid project_scan(const sensor_msgs::msg::LaserScan &scan,
                                        const LaserPose &pose, const GridParameters &parameters) {
  if (!std::isfinite(parameters.size) || !std::isfinite(parameters.resolution) ||
      parameters.size <= 0 || parameters.resolution <= 0 ||
      parameters.size / parameters.resolution > 1000 ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
    throw std::invalid_argument("invalid scan grid parameters");
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment == 0 ||
      !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0 || scan.range_max <= scan.range_min)
    throw std::invalid_argument("invalid laser scan metadata");

  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan.header;
  grid.header.frame_id = "odom";
  grid.info.map_load_time = scan.header.stamp;
  grid.info.resolution = static_cast<float>(parameters.resolution);
  grid.info.width = grid.info.height = static_cast<unsigned>(std::ceil(
      parameters.size / parameters.resolution));
  grid.info.origin.position.x = pose.x - grid.info.width * parameters.resolution / 2.0;
  grid.info.origin.position.y = pose.y - grid.info.height * parameters.resolution / 2.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, -1);

  auto cell = [&](double x, double y) -> int {
    const auto col = static_cast<int>(std::floor(
        (x - grid.info.origin.position.x) / parameters.resolution));
    const auto row = static_cast<int>(std::floor(
        (y - grid.info.origin.position.y) / parameters.resolution));
    if (col < 0 || row < 0 || col >= static_cast<int>(grid.info.width) ||
        row >= static_cast<int>(grid.info.height)) return -1;
    return row * static_cast<int>(grid.info.width) + col;
  };
  for (std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {
    const double measured = scan.ranges[beam];
    if (std::isnan(measured) || measured < scan.range_min) continue;
    const bool hit = std::isfinite(measured) && measured < scan.range_max;
    const double length = std::min(measured, static_cast<double>(scan.range_max));
    if (!(length > 0)) continue;
    const double angle = pose.yaw + scan.angle_min + beam * scan.angle_increment;
    const double dx = std::cos(angle), dy = std::sin(angle);
    // Полшага клетки достаточно для непрерывного луча; выход из окна заканчивает обработку.
    for (double range = 0; range < length; range += parameters.resolution / 2.0) {
      const int index = cell(pose.x + range * dx, pose.y + range * dy);
      if (index < 0) break;
      if (grid.data[index] != 100) grid.data[index] = 0;
    }
    if (hit) {
      const int index = cell(pose.x + length * dx, pose.y + length * dy);
      if (index >= 0) grid.data[index] = 100;
    }
  }
  return grid;
}

}  // namespace capstone_nav2
