#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace capstone_fleet {

struct Cell {
  int x{0};
  int y{0};
  bool operator==(const Cell &other) const { return x == other.x && y == other.y; }
  bool operator!=(const Cell &other) const { return !(*this == other); }
  bool operator<(const Cell &other) const { return y == other.y ? x < other.x : y < other.y; }
};

// origin задаёт центр клетки (0,0). Рёбра blocked_edges запрещены в обе стороны.
struct Grid {
  int width{0};
  int height{0};
  double resolution{1.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::uint8_t> blocked;
  std::vector<std::pair<Cell, Cell>> blocked_edges;

  void validate() const;
  bool free(Cell cell) const;
  bool traversable(Cell from, Cell to) const;
  std::pair<double, double> world(Cell cell) const;
};

using Path = std::vector<Cell>;
using Paths = std::vector<Path>;

enum class Method { CBS, ECBS };
enum class PlanStatus { Success, NoSolution, ResourceLimit, InvalidInput };
enum class ConflictKind { Vertex, SweptEdge };

struct Conflict {
  std::size_t first{0};
  std::size_t second{0};
  int time{0};
  ConflictKind kind{ConflictKind::Vertex};
  Cell first_from, first_to, second_from, second_to;
};

struct PlanOptions {
  Method method{Method::CBS};
  double weight{1.5};
  double clearance{0.85};
  int max_time_steps{64};
  std::size_t max_high_level_expansions{20000};
  std::size_t max_low_level_expansions{1000000};
  std::chrono::milliseconds wall_timeout{5000};
};

struct PlanResult {
  PlanStatus status{PlanStatus::InvalidInput};
  std::string reason;
  Paths paths;
  Paths independent_paths;
  std::vector<Conflict> initial_conflicts;
  std::vector<Conflict> resolved_conflicts;
  std::size_t final_conflicts{0};
  int cost{0};
  int lower_bound{0};
  double suboptimality_bound{1.0};
  std::size_t high_level_expansions{0};
  std::size_t low_level_expansions{0};
  std::size_t high_level_focal_slack_expansions{0};
  std::size_t low_level_focal_slack_expansions{0};
};

// Пути результата дополнены ожиданием до общего горизонта. Cost считает действия
// до последнего прибытия каждой машины, без добавленного конечного ожидания.
PlanResult plan(const Grid &grid, const std::vector<Cell> &starts,
                const std::vector<Cell> &goals, const PlanOptions &options = {});

// Конфликт объёмов рёбер учитывает любой порядок движения внутри общего раунда.
// Это более консервативно, чем проверка точечных агентов с одинаковой скоростью.
std::vector<Conflict> find_conflicts(const Grid &grid, const Paths &paths, double clearance);
double segment_distance(const Grid &grid, Cell first_from, Cell first_to,
                        Cell second_from, Cell second_to);

// Кратчайшее число шагов по статической карте; -1 означает недостижимую цель.
int shortest_distance(const Grid &grid, Cell start, Cell goal);

struct Assignment {
  std::size_t robot{0};
  std::size_t job{0};
  int cost{0};
};
struct AssignmentResult {
  std::vector<Assignment> assignments;
  int total_cost{0};
};

// Матрица robots × jobs. Отрицательная стоимость запрещает назначение.
// Сначала максимизируется число назначений, затем минимизируется сумма стоимости.
AssignmentResult minimum_cost_assignment(const std::vector<std::vector<int>> &costs);

}  // namespace capstone_fleet
