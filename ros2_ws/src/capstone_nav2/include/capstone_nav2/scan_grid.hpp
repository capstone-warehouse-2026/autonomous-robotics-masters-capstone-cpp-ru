#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace capstone_nav2 {

struct GridParameters {
  double size{8.0};
  double resolution{0.05};
};
struct LaserPose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Независимая от Nav2 inflation карта: только свободные лучи и измеренные препятствия.
nav_msgs::msg::OccupancyGrid project_scan(const sensor_msgs::msg::LaserScan &scan,
                                        const LaserPose &pose, const GridParameters &parameters);

}  // namespace capstone_nav2
