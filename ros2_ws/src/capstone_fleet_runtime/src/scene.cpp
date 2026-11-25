#include "capstone_fleet_runtime/scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace capstone_fleet_runtime {
namespace {
bool valid_id(const std::string &id) {
  return !id.empty() && !std::isdigit(static_cast<unsigned char>(id.front())) &&
      std::all_of(id.begin(), id.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_';
      });
}

bool capsule_is_free(const nav_msgs::msg::OccupancyGrid &map,
                     double ax, double ay, double bx, double by, double radius) {
  const auto &info = map.info;
  const double resolution = info.resolution;
  const double ox = info.origin.position.x, oy = info.origin.position.y;
  const double min_col = std::floor((std::min(ax, bx) - radius - ox) / resolution);
  const double max_col = std::floor((std::max(ax, bx) + radius - ox) / resolution);
  const double min_row = std::floor((std::min(ay, by) - radius - oy) / resolution);
  const double max_row = std::floor((std::max(ay, by) + radius - oy) / resolution);
  if (min_col < 0 || min_row < 0 || max_col >= info.width || max_row >= info.height) return false;
  const int xmin = static_cast<int>(min_col), xmax = static_cast<int>(max_col);
  const int ymin = static_cast<int>(min_row), ymax = static_cast<int>(max_row);
  // Рёбра решётки осевые. Расстояние от прямоугольника клетки карты до такого
  // отрезка вычисляется точно; неизвестные клетки считаются препятствиями.
  for (int y = ymin; y <= ymax; ++y) {
    for (int x = xmin; x <= xmax; ++x) {
      const auto value = map.data[static_cast<std::size_t>(y) * info.width + x];
      if (value >= 0 && value < 50) continue;
      const double left = ox + x * resolution, right = left + resolution;
      const double bottom = oy + y * resolution, top = bottom + resolution;
      const double dx = std::max({left - std::max(ax, bx), std::min(ax, bx) - right, 0.0});
      const double dy = std::max({bottom - std::max(ay, by), std::min(ay, by) - top, 0.0});
      if (std::hypot(dx, dy) <= radius) return false;
    }
  }
  return true;
}
}  // namespace

Scene load_scene(const std::string &path) {
  Scene scene;
  scene.mission = capstone_mission::load_scenario(path);
  const auto yaml = YAML::LoadFile(path);
  const auto grid = yaml["grid"];
  scene.grid.width = grid["width"].as<int>();
  scene.grid.height = grid["height"].as<int>();
  scene.grid.resolution = grid["resolution"].as<double>();
  scene.grid.origin_x = grid["origin_x"].as<double>();
  scene.grid.origin_y = grid["origin_y"].as<double>();
  if (scene.grid.width <= 0 || scene.grid.height <= 0 ||
      scene.grid.width > 200 || scene.grid.height > 200) {
    throw std::invalid_argument("fleet grid dimensions must be in 1..200");
  }
  scene.grid.blocked.assign(static_cast<std::size_t>(scene.grid.width) * scene.grid.height, 0);
  scene.grid.validate();
  std::set<std::string> ids;
  std::set<capstone_fleet::Cell> starts;
  for (const auto &robot : yaml["robots"]) {
    RobotSpec spec{robot["id"].as<std::string>(), robot["initial_station"].as<std::string>()};
    if (!valid_id(spec.id) || !ids.insert(spec.id).second ||
        !starts.insert(station_cell(scene, spec.initial_station)).second) {
      throw std::invalid_argument("robot ids and initial cells must be valid and unique");
    }
    scene.robots.push_back(spec);
  }
  if (scene.robots.size() < 2 || scene.robots.size() > 16) {
    throw std::invalid_argument("fleet scenario requires 2..16 robots");
  }
  for (const auto &station : scene.mission.stations) station_cell(scene, station.id);
  return scene;
}

capstone_fleet::Cell station_cell(const Scene &scene, const std::string &station) {
  const auto &point = capstone_mission::station_by_id(scene.mission, station);
  const double x = (point.x - scene.grid.origin_x) / scene.grid.resolution;
  const double y = (point.y - scene.grid.origin_y) / scene.grid.resolution;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
      x >= scene.grid.width || y >= scene.grid.height) {
    throw std::invalid_argument("station outside fleet grid: " + station);
  }
  const capstone_fleet::Cell cell{static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y))};
  if (std::abs(x - cell.x) > 1e-6 || std::abs(y - cell.y) > 1e-6 ||
      cell.x < 0 || cell.y < 0 || cell.x >= scene.grid.width || cell.y >= scene.grid.height) {
    throw std::invalid_argument("station must coincide with a fleet grid cell: " + station);
  }
  return cell;
}

capstone_fleet::Grid grid_from_map(const capstone_fleet::Grid &lattice,
                                  const nav_msgs::msg::OccupancyGrid &map, double radius) {
  lattice.validate();
  const auto &info = map.info;
  const auto &rotation = info.origin.orientation;
  if (map.header.frame_id != "map" || info.width == 0 || info.height == 0 ||
      !std::isfinite(info.resolution) || info.resolution <= 0 ||
      !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
      map.data.size() != static_cast<std::size_t>(info.width) * info.height ||
      map.data.size() > 10000000U ||
      !std::isfinite(radius) || radius <= 0 ||
      !std::isfinite(rotation.x) || !std::isfinite(rotation.y) ||
      !std::isfinite(rotation.z) || !std::isfinite(rotation.w) ||
      std::abs(rotation.x) > 1e-9 || std::abs(rotation.y) > 1e-9 ||
      std::abs(rotation.z) > 1e-9 || std::abs(rotation.w - 1) > 1e-9) {
    throw std::invalid_argument("invalid or rotated static map");
  }
  auto grid = lattice;
  grid.blocked_edges.clear();
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const auto [wx, wy] = grid.world({x, y});
      grid.blocked[static_cast<std::size_t>(y) * grid.width + x] =
          !capsule_is_free(map, wx, wy, wx, wy, radius);
    }
  }
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const capstone_fleet::Cell from{x, y};
      if (!grid.free(from)) continue;
      for (const capstone_fleet::Cell to : {capstone_fleet::Cell{x + 1, y}, {x, y + 1}}) {
        if (!grid.free(to)) continue;
        const auto [ax, ay] = grid.world(from);
        const auto [bx, by] = grid.world(to);
        if (!capsule_is_free(map, ax, ay, bx, by, radius)) grid.blocked_edges.emplace_back(from, to);
      }
    }
  }
  return grid;
}
}  // namespace capstone_fleet_runtime
