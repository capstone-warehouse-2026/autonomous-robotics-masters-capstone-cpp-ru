#include "capstone_fleet/planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace capstone_fleet {
namespace {
constexpr double kEpsilon = 1e-9;
const std::array<Cell, 5> kMoves{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {0, 0}}};
using Point = std::pair<double, double>;

double squared_distance(Point first, Point second) {
  const double x = first.first - second.first, y = first.second - second.second;
  return x * x + y * y;
}

double point_segment_distance(Point point, Point start, Point end) {
  const double length = squared_distance(start, end);
  if (length < kEpsilon) return std::sqrt(squared_distance(point, start));
  const double position = std::clamp(((point.first - start.first) * (end.first - start.first) +
      (point.second - start.second) * (end.second - start.second)) / length, 0.0, 1.0);
  const Point closest{start.first + position * (end.first - start.first),
                      start.second + position * (end.second - start.second)};
  return std::sqrt(squared_distance(point, closest));
}

double cross(Point first, Point second, Point third) {
  return (second.first - first.first) * (third.second - first.second) -
         (second.second - first.second) * (third.first - first.first);
}

double segments_distance(Point a, Point b, Point c, Point d) {
  // Строгое пересечение. Коллинеарные отрезки и вырожденные ожидания покрывает
  // последующая проверка расстояний от концов к противоположному отрезку.
  const double first = cross(a, b, c), second = cross(a, b, d);
  const double third = cross(c, d, a), fourth = cross(c, d, b);
  if (((first > 0 && second < 0) || (first < 0 && second > 0)) &&
      ((third > 0 && fourth < 0) || (third < 0 && fourth > 0))) return 0.0;
  return std::min({point_segment_distance(a, c, d), point_segment_distance(b, c, d),
                   point_segment_distance(c, a, b), point_segment_distance(d, a, b)});
}

Cell position_at(const Path &path, int time) {
  return path.at(std::min(static_cast<std::size_t>(time), path.size() - 1));
}

std::vector<int> distances_to(const Grid &grid, Cell goal) {
  std::vector<int> distance(static_cast<std::size_t>(grid.width) * grid.height, -1);
  if (!grid.free(goal)) return distance;
  std::queue<Cell> pending;
  pending.push(goal);
  distance[goal.y * grid.width + goal.x] = 0;
  while (!pending.empty()) {
    const auto current = pending.front();
    pending.pop();
    for (std::size_t move = 0; move < 4; ++move) {
      const Cell next{current.x + kMoves[move].x, current.y + kMoves[move].y};
      if (!grid.traversable(current, next)) continue;
      const auto index = next.y * grid.width + next.x;
      if (distance[index] >= 0) continue;
      distance[index] = distance[current.y * grid.width + current.x] + 1;
      pending.push(next);
    }
  }
  return distance;
}

struct Constraint {
  std::size_t agent;
  ConflictKind kind;
  int time;
  Cell from, to;
};

bool permits_start(Cell start, const std::vector<Constraint> &constraints) {
  return std::none_of(constraints.begin(), constraints.end(), [start](const auto &constraint) {
    return constraint.kind == ConflictKind::Vertex && constraint.time == 0 &&
           constraint.to == start;
  });
}

bool permits_transition(Cell from, Cell to, int time, const std::vector<Constraint> &constraints) {
  return std::none_of(constraints.begin(), constraints.end(), [&](const auto &constraint) {
    if (constraint.kind == ConflictKind::Vertex)
      return constraint.time == time + 1 && constraint.to == to;
    return constraint.time == time && constraint.from == from && constraint.to == to;
  });
}

bool permits_goal_wait(Cell goal, int time, const std::vector<Constraint> &constraints) {
  return std::none_of(constraints.begin(), constraints.end(), [&](const auto &constraint) {
    if (constraint.time < time) return false;
    if (constraint.kind == ConflictKind::Vertex) return constraint.to == goal;
    return constraint.from == goal && constraint.to == goal;
  });
}

struct SearchBudget {
  const PlanOptions &options;
  std::chrono::steady_clock::time_point deadline;
  std::size_t low_expansions{0};
  std::size_t high_expansions{0};
  std::size_t low_focal_slack_expansions{0};
  std::size_t high_focal_slack_expansions{0};
  std::string reason;

  bool available(bool low_level) {
    if (std::chrono::steady_clock::now() >= deadline) reason = "wall_timeout";
    else if (low_level && low_expansions >= options.max_low_level_expansions)
      reason = "low_level_expansion_limit";
    else if (!low_level && high_expansions >= options.max_high_level_expansions)
      reason = "high_level_expansion_limit";
    return reason.empty();
  }
};

struct LowNode {
  Cell cell;
  int time;
  int heuristic;
  int conflicts;
  std::size_t parent;
  bool open{true};
};

enum class LowStatus { Found, NoPath, Limit };
struct LowResult {
  LowStatus status{LowStatus::NoPath};
  Path path;
  int lower_bound{0};
};

// OPEN всегда упорядочен по допустимому f=g+h. FOCAL выбирает внутри порога
// по числу мягких конфликтов, а не превращает h в взвешенную эвристику A*.
using OpenKey = std::tuple<int, int, int, std::size_t>;
OpenKey open_key(const LowNode &node, std::size_t index) {
  return {node.time + node.heuristic, node.conflicts, node.heuristic, index};
}

int transition_conflicts(const Grid &grid, Cell from, Cell to, int time,
                         std::size_t agent, const Paths &others, double clearance) {
  int conflicts = 0;
  for (std::size_t other = 0; other < others.size(); ++other) {
    if (other == agent || others[other].empty()) continue;
    if (segment_distance(grid, from, to, position_at(others[other], time),
                         position_at(others[other], time + 1)) + kEpsilon < clearance) ++conflicts;
  }
  return conflicts;
}

LowResult find_path(const Grid &grid, Cell start, Cell goal, std::size_t agent,
                    const std::vector<Constraint> &all_constraints, const Paths &others,
                    double weight, SearchBudget &budget) {
  std::vector<Constraint> constraints;
  for (const auto &constraint : all_constraints)
    if (constraint.agent == agent) constraints.push_back(constraint);
  const auto heuristic = distances_to(grid, goal);
  const int initial_h = heuristic[start.y * grid.width + start.x];
  if (initial_h < 0 || !permits_start(start, constraints)) return {};

  const std::size_t none = std::numeric_limits<std::size_t>::max();
  std::vector<LowNode> nodes{{start, 0, initial_h, 0, none, true}};
  std::set<OpenKey> open{open_key(nodes.front(), 0)};
  std::unordered_map<std::int64_t, std::size_t> state_index;
  const auto state_key = [&](Cell cell, int time) {
    return (static_cast<std::int64_t>(time) * grid.height + cell.y) * grid.width + cell.x;
  };
  state_index[state_key(start, 0)] = 0;
  while (!open.empty()) {
    if (!budget.available(true)) return {LowStatus::Limit, {}, 0};
    const int minimum_f = std::get<0>(*open.begin());
    auto selected = open.begin();
    for (auto candidate = open.begin(); candidate != open.end(); ++candidate) {
      if (std::get<0>(*candidate) > weight * minimum_f + kEpsilon) break;
      const auto candidate_rank = std::make_tuple(std::get<1>(*candidate), std::get<0>(*candidate),
                                                  std::get<2>(*candidate), std::get<3>(*candidate));
      const auto selected_rank = std::make_tuple(std::get<1>(*selected), std::get<0>(*selected),
                                                 std::get<2>(*selected), std::get<3>(*selected));
      if (candidate_rank < selected_rank) selected = candidate;
    }
    const auto index = std::get<3>(*selected);
    open.erase(selected);
    nodes[index].open = false;
    const auto current = nodes[index];
    ++budget.low_expansions;
    if (current.time + current.heuristic > minimum_f) ++budget.low_focal_slack_expansions;
    if (current.cell == goal && permits_goal_wait(goal, current.time, constraints)) {
      Path path;
      for (auto cursor = index; cursor != none; cursor = nodes[cursor].parent)
        path.push_back(nodes[cursor].cell);
      std::reverse(path.begin(), path.end());
      return {LowStatus::Found, std::move(path), minimum_f};
    }
    if (current.time >= budget.options.max_time_steps) continue;
    for (const auto move : kMoves) {
      const Cell next{current.cell.x + move.x, current.cell.y + move.y};
      if (!grid.traversable(current.cell, next) ||
          !permits_transition(current.cell, next, current.time, constraints)) continue;
      const int remaining = heuristic[next.y * grid.width + next.x];
      if (remaining < 0 || current.time + 1 + remaining > budget.options.max_time_steps) continue;
      const int conflicts = current.conflicts + transition_conflicts(
          grid, current.cell, next, current.time, agent, others, budget.options.clearance);
      const auto key = state_key(next, current.time + 1);
      const auto found = state_index.find(key);
      if (found != state_index.end()) {
        auto &existing = nodes[found->second];
        if (existing.conflicts <= conflicts) continue;
        if (existing.open) open.erase(open_key(existing, found->second));
        existing.conflicts = conflicts;
        existing.parent = index;
        existing.open = true;
        open.insert(open_key(existing, found->second));
      } else {
        const auto next_index = nodes.size();
        nodes.push_back({next, current.time + 1, remaining, conflicts, index, true});
        state_index[key] = next_index;
        open.insert(open_key(nodes.back(), next_index));
      }
    }
  }
  return {};
}

struct ConstraintTreeNode {
  std::vector<Constraint> constraints;
  Paths paths;
  std::vector<int> lower_bounds;
  std::vector<Conflict> conflicts;
  int cost{0};
  int lower_bound{0};
  std::size_t serial{0};
};

void update_node(const Grid &grid, ConstraintTreeNode &node, double clearance) {
  node.cost = 0;
  node.lower_bound = 0;
  for (std::size_t agent = 0; agent < node.paths.size(); ++agent) {
    node.cost += static_cast<int>(node.paths[agent].size()) - 1;
    node.lower_bound += node.lower_bounds[agent];
  }
  node.conflicts = find_conflicts(grid, node.paths, clearance);
}

Constraint split_constraint(const Conflict &conflict, bool first) {
  return {first ? conflict.first : conflict.second, conflict.kind, conflict.time,
          first ? conflict.first_from : conflict.second_from,
          first ? conflict.first_to : conflict.second_to};
}

void pad_paths(Paths &paths) {
  std::size_t size = 0;
  for (const auto &path : paths) size = std::max(size, path.size());
  for (auto &path : paths) path.resize(size, path.back());
}
}  // namespace

void Grid::validate() const {
  const auto size = static_cast<std::int64_t>(width) * height;
  if (width <= 0 || height <= 0 || size > 1000000 ||
      blocked.size() != static_cast<std::size_t>(size) ||
      !std::isfinite(resolution) || resolution <= 0 ||
      !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
      !std::isfinite(origin_x + resolution * (width - 1)) ||
      !std::isfinite(origin_y + resolution * (height - 1)))
    throw std::invalid_argument("invalid grid dimensions, occupancy or geometry");
  for (const auto &[from, to] : blocked_edges) {
    const auto inside = [this](Cell cell) {
      return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
    };
    if (!inside(from) || !inside(to) || std::abs(from.x - to.x) + std::abs(from.y - to.y) != 1)
      throw std::invalid_argument("blocked edge must connect adjacent grid cells");
  }
}

bool Grid::free(Cell cell) const {
  return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height &&
         blocked.at(static_cast<std::size_t>(cell.y) * width + cell.x) == 0;
}

bool Grid::traversable(Cell from, Cell to) const {
  if (!free(from) || !free(to) || std::abs(from.x - to.x) + std::abs(from.y - to.y) > 1) return false;
  return std::none_of(blocked_edges.begin(), blocked_edges.end(), [&](const auto &edge) {
    return (edge.first == from && edge.second == to) || (edge.second == from && edge.first == to);
  });
}

std::pair<double, double> Grid::world(Cell cell) const {
  return {origin_x + resolution * cell.x, origin_y + resolution * cell.y};
}

double segment_distance(const Grid &grid, Cell first_from, Cell first_to,
                        Cell second_from, Cell second_to) {
  return segments_distance(grid.world(first_from), grid.world(first_to),
                           grid.world(second_from), grid.world(second_to));
}

std::vector<Conflict> find_conflicts(const Grid &grid, const Paths &paths, double clearance) {
  if (!std::isfinite(clearance) || clearance <= 0) throw std::invalid_argument("invalid clearance");
  std::size_t horizon = 0;
  for (const auto &path : paths) {
    if (path.empty()) throw std::invalid_argument("empty path");
    horizon = std::max(horizon, path.size());
  }
  std::vector<Conflict> conflicts;
  for (std::size_t first = 0; first < paths.size(); ++first) {
    for (std::size_t second = first + 1; second < paths.size(); ++second) {
      const auto a = paths[first].front(), b = paths[second].front();
      if (segment_distance(grid, a, a, b, b) + kEpsilon < clearance)
        conflicts.push_back({first, second, 0, ConflictKind::Vertex, a, a, b, b});
    }
  }
  for (int time = 0; time + 1 < static_cast<int>(horizon); ++time) {
    for (std::size_t first = 0; first < paths.size(); ++first) {
      for (std::size_t second = first + 1; second < paths.size(); ++second) {
        const auto a = position_at(paths[first], time), b = position_at(paths[first], time + 1);
        const auto c = position_at(paths[second], time), d = position_at(paths[second], time + 1);
        if (segment_distance(grid, b, b, d, d) + kEpsilon < clearance)
          conflicts.push_back({first, second, time + 1, ConflictKind::Vertex, b, b, d, d});
        else if (segment_distance(grid, a, b, c, d) + kEpsilon < clearance)
          conflicts.push_back({first, second, time, ConflictKind::SweptEdge, a, b, c, d});
      }
    }
  }
  return conflicts;
}

int shortest_distance(const Grid &grid, Cell start, Cell goal) {
  grid.validate();
  if (!grid.free(start) || !grid.free(goal)) return -1;
  return distances_to(grid, goal)[start.y * grid.width + start.x];
}

PlanResult plan(const Grid &grid, const std::vector<Cell> &starts,
                const std::vector<Cell> &goals, const PlanOptions &options) {
  PlanResult result;
  try {
    grid.validate();
    if (starts.empty() || starts.size() != goals.size() || starts.size() > 32 ||
        (options.method != Method::CBS && options.method != Method::ECBS) ||
        !std::isfinite(options.weight) || options.weight < 1 ||
        !std::isfinite(options.clearance) || options.clearance <= 0 ||
        options.max_time_steps < 0 || options.max_time_steps > 4096 ||
        options.wall_timeout.count() <= 0 || options.max_high_level_expansions == 0 ||
        options.max_low_level_expansions == 0)
      throw std::invalid_argument("invalid agents or planner limits");
    for (std::size_t agent = 0; agent < starts.size(); ++agent)
      if (!grid.free(starts[agent]) || !grid.free(goals[agent]))
        throw std::invalid_argument("start or goal occupies blocked cell");
  } catch (const std::invalid_argument &error) {
    result.reason = error.what();
    return result;
  }
  for (std::size_t first = 0; first < starts.size(); ++first) {
    for (std::size_t second = first + 1; second < starts.size(); ++second) {
      if (segment_distance(grid, starts[first], starts[first], starts[second], starts[second]) +
          kEpsilon < options.clearance) {
        result.reason = "start positions overlap";
        return result;
      }
      if (segment_distance(grid, goals[first], goals[first], goals[second], goals[second]) +
          kEpsilon < options.clearance) {
        result.status = PlanStatus::NoSolution;
        result.reason = "permanent goal occupancies overlap";
        return result;
      }
    }
  }
  SearchBudget budget{options, std::chrono::steady_clock::now() + options.wall_timeout,
                       0, 0, 0, 0, {}};
  const double weight = options.method == Method::CBS ? 1.0 : options.weight;
  result.suboptimality_bound = weight;
  auto root = std::make_shared<ConstraintTreeNode>();
  root->paths.resize(starts.size());
  root->lower_bounds.resize(starts.size());
  auto finish = [&](PlanStatus status, const std::string &reason) {
    result.status = status;
    result.reason = reason;
    result.low_level_expansions = budget.low_expansions;
    result.high_level_expansions = budget.high_expansions;
    result.low_level_focal_slack_expansions = budget.low_focal_slack_expansions;
    result.high_level_focal_slack_expansions = budget.high_focal_slack_expansions;
    return result;
  };
  // Единый независимый A* baseline позволяет увидеть исходные конфликты у обоих
  // алгоритмов. Начальный CT имеет точные нижние границы; ECBS ослабляет replanning.
  for (std::size_t agent = 0; agent < starts.size(); ++agent) {
    const auto low = find_path(grid, starts[agent], goals[agent], agent, {}, {}, 1.0, budget);
    if (low.status == LowStatus::Limit) return finish(PlanStatus::ResourceLimit, budget.reason);
    if (low.status != LowStatus::Found) return finish(PlanStatus::NoSolution, "no independent path within horizon");
    root->paths[agent] = low.path;
    root->lower_bounds[agent] = low.lower_bound;
  }
  update_node(grid, *root, options.clearance);
  result.independent_paths = root->paths;
  pad_paths(result.independent_paths);
  result.initial_conflicts = root->conflicts;
  std::vector<std::shared_ptr<ConstraintTreeNode>> open{root};
  std::size_t next_serial = 1;
  while (!open.empty()) {
    if (!budget.available(false)) return finish(PlanStatus::ResourceLimit, budget.reason);
    int minimum_bound = std::numeric_limits<int>::max();
    for (const auto &node : open) minimum_bound = std::min(minimum_bound, node->lower_bound);
    std::optional<std::size_t> selected;
    for (std::size_t candidate = 0; candidate < open.size(); ++candidate) {
      const auto &node = *open[candidate];
      if (node.cost > weight * minimum_bound + kEpsilon) continue;
      if (!selected || std::make_tuple(node.conflicts.size(), node.cost, node.serial) <
          std::make_tuple(open[*selected]->conflicts.size(), open[*selected]->cost,
                          open[*selected]->serial)) selected = candidate;
    }
    if (!selected) throw std::logic_error("ECBS FOCAL invariant violated");
    const auto current = open[*selected];
    open.erase(open.begin() + static_cast<std::ptrdiff_t>(*selected));
    ++budget.high_expansions;
    if (current->cost > minimum_bound) ++budget.high_focal_slack_expansions;
    if (current->conflicts.empty()) {
      result.paths = current->paths;
      pad_paths(result.paths);
      result.cost = current->cost;
      result.lower_bound = minimum_bound;
      result.final_conflicts = find_conflicts(grid, result.paths, options.clearance).size();
      return finish(PlanStatus::Success, "conflict-free plan");
    }
    const auto conflict = current->conflicts.front();
    result.resolved_conflicts.push_back(conflict);
    for (const bool first : {true, false}) {
      auto child = std::make_shared<ConstraintTreeNode>(*current);
      const auto constraint = split_constraint(conflict, first);
      child->constraints.push_back(constraint);
      const auto agent = constraint.agent;
      const auto low = find_path(grid, starts[agent], goals[agent], agent,
                                 child->constraints, child->paths, weight, budget);
      // Лимит поиска не означает невозможность: нельзя отбрасывать такую ветвь
      // и продолжать заявлять оптимальность или ограниченную субоптимальность.
      if (low.status == LowStatus::Limit) return finish(PlanStatus::ResourceLimit, budget.reason);
      if (low.status != LowStatus::Found) continue;
      child->paths[agent] = low.path;
      child->lower_bounds[agent] = low.lower_bound;
      child->serial = next_serial++;
      update_node(grid, *child, options.clearance);
      open.push_back(std::move(child));
    }
  }
  return finish(PlanStatus::NoSolution, "constraint tree exhausted within horizon");
}
}  // namespace capstone_fleet
