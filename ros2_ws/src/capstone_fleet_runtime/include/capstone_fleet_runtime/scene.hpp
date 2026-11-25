#pragma once

#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "capstone_fleet/planner.hpp"
#include "capstone_mission/domain.hpp"

namespace capstone_fleet_runtime {
struct RobotSpec {
  std::string id;
  std::string initial_station;
};
struct Scene {
  capstone_mission::Scenario mission;
  capstone_fleet::Grid grid;
  std::vector<RobotSpec> robots;
};
Scene load_scene(const std::string &path);
capstone_fleet::Cell station_cell(const Scene &scene, const std::string &station);
capstone_fleet::Grid grid_from_map(const capstone_fleet::Grid &lattice,
                                  const nav_msgs::msg::OccupancyGrid &map, double radius);
}  // namespace capstone_fleet_runtime
