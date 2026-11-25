#include "capstone_fleet/planner.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace capstone_fleet {
namespace {
struct Edge {
  int to;
  int reverse;
  int capacity;
  int cost;
};
void add_edge(std::vector<std::vector<Edge>> &graph, int from, int to, int cost) {
  const int from_reverse = static_cast<int>(graph[to].size());
  const int to_reverse = static_cast<int>(graph[from].size());
  graph[from].push_back({to, from_reverse, 1, cost});
  graph[to].push_back({from, to_reverse, 0, -cost});
}
}  // namespace

AssignmentResult minimum_cost_assignment(const std::vector<std::vector<int>> &costs) {
  AssignmentResult result;
  if (costs.empty()) return result;
  const auto robots = costs.size(), jobs = costs.front().size();
  if (robots > 1000 || jobs > 1000) throw std::invalid_argument("assignment matrix too large");
  for (const auto &row : costs)
    if (row.size() != jobs) throw std::invalid_argument("ragged assignment matrix");
  if (jobs == 0) return result;
  const int source = 0, first_robot = 1, first_job = first_robot + static_cast<int>(robots);
  const int sink = first_job + static_cast<int>(jobs), vertices = sink + 1;
  std::vector<std::vector<Edge>> graph(vertices);
  for (std::size_t robot = 0; robot < robots; ++robot) {
    add_edge(graph, source, first_robot + static_cast<int>(robot), 0);
    for (std::size_t job = 0; job < jobs; ++job)
      if (costs[robot][job] >= 0)
        add_edge(graph, first_robot + static_cast<int>(robot), first_job + static_cast<int>(job),
                 costs[robot][job]);
  }
  for (std::size_t job = 0; job < jobs; ++job)
    add_edge(graph, first_job + static_cast<int>(job), sink, 0);

  // Последовательные кратчайшие увеличивающие пути дают maximum cardinality,
  // затем minimum cost. Обратные рёбра позволяют исправлять раннее назначение.
  const auto infinity = std::numeric_limits<long long>::max() / 4;
  while (true) {
    std::vector<long long> distance(vertices, infinity);
    std::vector<int> parent(vertices, -1), parent_edge(vertices, -1);
    distance[source] = 0;
    for (int iteration = 0; iteration + 1 < vertices; ++iteration) {
      bool changed = false;
      for (int from = 0; from < vertices; ++from) {
        if (distance[from] == infinity) continue;
        for (std::size_t edge_index = 0; edge_index < graph[from].size(); ++edge_index) {
          const auto &edge = graph[from][edge_index];
          if (!edge.capacity || distance[edge.to] <= distance[from] + edge.cost) continue;
          distance[edge.to] = distance[from] + edge.cost;
          parent[edge.to] = from;
          parent_edge[edge.to] = static_cast<int>(edge_index);
          changed = true;
        }
      }
      if (!changed) break;
    }
    if (parent[sink] == -1) break;
    for (int vertex = sink; vertex != source; vertex = parent[vertex]) {
      auto &edge = graph[parent[vertex]][parent_edge[vertex]];
      --edge.capacity;
      ++graph[vertex][edge.reverse].capacity;
    }
  }
  long long total = 0;
  for (std::size_t robot = 0; robot < robots; ++robot) {
    for (const auto &edge : graph[first_robot + static_cast<int>(robot)]) {
      if (edge.to < first_job || edge.to >= sink || edge.capacity != 0) continue;
      const auto job = static_cast<std::size_t>(edge.to - first_job);
      result.assignments.push_back({robot, job, costs[robot][job]});
      total += costs[robot][job];
    }
  }
  if (total > std::numeric_limits<int>::max()) throw std::overflow_error("assignment total cost overflow");
  result.total_cost = static_cast<int>(total);
  return result;
}
}  // namespace capstone_fleet
