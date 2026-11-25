#include "capstone_planning/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace capstone_planning {

std::vector<Pose2D> cells_to_poses(const Grid &grid, const std::vector<Cell> &cells, const Pose2D &start,
                                   const Pose2D &goal) {
  std::vector<Pose2D> poses;
  poses.reserve(cells.size() + 2);
  poses.push_back(start);
  // Interior cells only: the start and goal cells are represented by the exact poses.
  for (std::size_t i = 1; i + 1 < cells.size(); ++i) {
    const Point2D p = grid.cell_center(cells[i]);
    poses.push_back({p.x, p.y, 0.0});
  }
  poses.push_back(goal);
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    const double dx = poses[i + 1].x - poses[i].x;
    const double dy = poses[i + 1].y - poses[i].y;
    poses[i].yaw = (dx == 0.0 && dy == 0.0) ? (i > 0 ? poses[i - 1].yaw : start.yaw) : std::atan2(dy, dx);
  }
  poses.back().yaw = goal.yaw;
  return poses;
}

double path_length(const std::vector<Pose2D> &poses) {
  double length = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    length += std::hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
  }
  return length;
}

std::vector<Point2D> sample_path(const std::vector<Pose2D> &poses, double step) {
  std::vector<Point2D> points;
  if (poses.empty()) return points;
  points.push_back({poses.front().x, poses.front().y});
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double dx = poses[i].x - poses[i - 1].x;
    const double dy = poses[i].y - poses[i - 1].y;
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / step)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      points.push_back({poses[i - 1].x + t * dx, poses[i - 1].y + t * dy});
    }
  }
  return points;
}

ClearanceStats path_clearance(const CostMap &map, const std::vector<Pose2D> &poses) {
  ClearanceStats stats;
  std::vector<double> values;
  for (const Point2D &p : sample_path(poses, 0.5 * map.resolution())) {
    const Cell c = map.grid().cell_at(p);
    if (map.grid().contains(c)) values.push_back(map.clearance(c));
  }
  if (values.empty()) return stats;
  std::sort(values.begin(), values.end());
  stats.min = values.front();
  stats.p5 = values[static_cast<std::size_t>(std::floor(0.05 * static_cast<double>(values.size() - 1)))];
  return stats;
}

bool path_blocked(const CostMap &map, const std::vector<Pose2D> &poses) {
  bool reached_free = false;
  for (const Point2D &p : sample_path(poses, 0.5 * map.resolution())) {
    const Cell c = map.grid().cell_at(p);
    const bool ok = map.traversable(c);
    if (!reached_free) {
      if (!map.grid().contains(c) || map.obstacle(c)) return true;
      reached_free = ok;
      continue;
    }
    if (!ok) return true;
  }
  return false;
}

PlannedPath plan_path(const CostMap &map, const Pose2D &start, const Pose2D &goal, const SearchOptions &options) {
  const auto started = std::chrono::steady_clock::now();
  PlannedPath out;
  const Grid &grid = map.grid();
  SearchResult search = plan_on_grid(map, grid.cell_at({start.x, start.y}), grid.cell_at({goal.x, goal.y}), options);
  out.status = search.status;
  out.expanded = search.expanded;
  if (search.status == PlanStatus::kSuccess) {
    out.cost = search.cost;
    out.poses = cells_to_poses(grid, search.cells, start, goal);
    out.length = path_length(out.poses);
    const ClearanceStats clearance = path_clearance(map, out.poses);
    out.min_clearance = clearance.min;
    out.p5_clearance = clearance.p5;
  }
  out.planning_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  return out;
}

}  // namespace capstone_planning
