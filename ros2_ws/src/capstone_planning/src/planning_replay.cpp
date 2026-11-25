// Offline replay of recorded or exported planning inputs through the C++ planner (no ROS graph):
//   planning_replay <episode_dir> <out_file> classical|learned [model.onnx]
//
// <episode_dir> (written by ml/planning/evaluate.py, or by a future rosbag exporter):
//   meta.txt        width height resolution origin_x origin_y
//   static.bin      int8 /map cells (row-major, width * height)
//   observed_K.bin  uint8 observed obstacles at t0 - K s, K = 0..3 (1 = occupied)
//   area.bin        uint8 area observed at t0
//   queries.txt     one "start_x start_y goal_x goal_y" per line
// <out_file>: per query one line "status cost expanded planning_ms x0 y0 x1 y1 ...".
// The cost map, mask and search are exactly those of planning_node for the chosen method.

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/learned_cost.hpp"
#include "capstone_planning/map_fusion.hpp"
#include "capstone_planning/planner.hpp"

namespace {

using namespace capstone_planning;

template <typename T>
std::vector<T> read_binary(const std::string &path, std::size_t count) {
  std::ifstream in(path, std::ios::binary);
  std::vector<T> data(count);
  if (!in.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(count * sizeof(T)))) {
    throw std::runtime_error("cannot read " + path);
  }
  return data;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: planning_replay <episode_dir> <out_file> classical|learned [model.onnx]\n";
    return 2;
  }
  const std::string dir = argv[1];
  const std::string method = argv[3];
  try {
    std::ifstream meta(dir + "/meta.txt");
    Grid map;
    meta >> map.width >> map.height >> map.resolution >> map.origin.x >> map.origin.y;
    if (!meta) throw std::runtime_error("cannot read meta.txt");
    map.data = read_binary<std::int8_t>(dir + "/static.bin", static_cast<std::size_t>(map.width) * map.height);

    // Frames t0 - 3 s .. t0 as fused maps: observed obstacles on top of the static map.
    const double t0 = 100.0;
    ObservationHistory history(1.0);
    FusedMap current;
    for (int k = LearnedLayout::kHistory - 1; k >= 0; --k) {
      FusedMap f;
      f.grid = map;
      f.observed_obstacle = read_binary<std::uint8_t>(dir + "/observed_" + std::to_string(k) + ".bin", map.size());
      f.observed_area = read_binary<std::uint8_t>(dir + "/area.bin", map.size());
      for (std::size_t i = 0; i < map.size(); ++i) {
        if (f.observed_obstacle[i] && !is_occupied_value(map.data[i])) {
          f.grid.data[i] = kOccupied;
        } else {
          f.observed_obstacle[i] = 0;
        }
      }
      history.push(t0 - k, f);
      if (k == 0) current = f;
    }

    std::shared_ptr<CostMap> cost_map;
    CostParams params;
    if (method == "classical") {
      cost_map = std::make_shared<CostMap>(current, params);
    } else if (method == "learned") {
      if (argc < 5) throw std::runtime_error("learned needs a model path");
      CostModel model(argv[4], 1);
      LearnedLayout layout;
      layout.block = model.info().block;
      layout.history_period = model.info().history_period;
      int h = 0, w = 0;
      const auto input = build_input(current, t0, history, model.info().history, layout, h, w);
      const auto extra = extra_cost_from_output(model.run(input, h, w), w, h, current.grid, current.observed_area,
                                                layout.block, model.info().unobserved_prior,
                                                model.info().max_extra_cost);
      params.observed_weight = 0.0;  // as planning_node: the learned cost replaces A's observed term
      cost_map = std::make_shared<CostMap>(current, params, &extra);
    } else {
      throw std::runtime_error("unknown method " + method);
    }

    std::ifstream queries(dir + "/queries.txt");
    std::ofstream out(argv[2]);
    double sx, sy, gx, gy;
    while (queries >> sx >> sy >> gx >> gy) {
      SearchOptions options;
      options.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      const PlannedPath p = plan_path(*cost_map, {sx, sy, 0.0}, {gx, gy, 0.0}, options);
      out << static_cast<int>(p.status) << ' ' << p.cost << ' ' << p.expanded << ' ' << p.planning_ms;
      for (const Pose2D &pose : p.poses) out << ' ' << pose.x << ' ' << pose.y;
      out << '\n';
    }
  } catch (const std::exception &e) {
    std::cerr << "planning_replay: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
