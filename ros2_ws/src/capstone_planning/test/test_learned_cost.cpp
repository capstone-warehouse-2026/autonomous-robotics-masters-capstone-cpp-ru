// Tests of method B without ROS: input/output conventions (ml/planning/sim.py), the released model
// against its golden output from PyTorch, fallbacks on invalid models, and that the learned cost keeps
// the hard mask and the admissibility of the A* heuristic.

#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_planning/cost_map.hpp"
#include "capstone_planning/grid_search.hpp"
#include "capstone_planning/learned_cost.hpp"
#include "capstone_planning/map_fusion.hpp"
#include "capstone_planning/scenarios.hpp"

namespace {

using namespace capstone_planning;
using scenarios::make_grid;

const std::string kModels = std::string(CAPSTONE_PLANNING_SOURCE_DIR) + "/models/";

FusedMap fused_with_observation(int width, int height) {
  FusedMap f;
  f.grid = make_grid(width * 0.05, height * 0.05, 0.05);
  f.observed_obstacle.assign(f.grid.size(), 0);
  f.observed_area.assign(f.grid.size(), 0);
  return f;
}

TEST(LearnedInput, PoolFractionWithPartialBlocks) {
  // 5 x 3 fine cells, block 4: coarse 2 x 1; the right block has 1 x 3 cells.
  std::vector<std::uint8_t> fine = {1, 1, 0, 0, 1,
                                    0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0};
  const auto c = pool_fraction(fine, 5, 3, 4);
  ASSERT_EQ(c.size(), 2u);
  EXPECT_FLOAT_EQ(c[0], 2.0f / 12.0f);
  EXPECT_FLOAT_EQ(c[1], 2.0f / 3.0f);
  EXPECT_THROW(pool_fraction(fine, 4, 3, 4), std::invalid_argument);
}

TEST(LearnedInput, ChannelsFollowSimObservation) {
  FusedMap f = fused_with_observation(8, 8);
  f.grid.data[f.grid.index(0, 0)] = kOccupied;   // static obstacle
  f.grid.data[f.grid.index(1, 0)] = kUnknown;    // unknown counts as static
  f.grid.data[f.grid.index(4, 4)] = kOccupied;   // observed only: not static
  f.observed_obstacle[f.grid.index(4, 4)] = 1;
  for (int y = 0; y < 8; ++y)
    for (int x = 4; x < 8; ++x) f.observed_area[f.grid.index(x, y)] = 1;

  ObservationHistory history(1.0);
  FusedMap past = f;
  past.observed_obstacle.assign(f.grid.size(), 0);
  past.observed_obstacle[f.grid.index(7, 7)] = 1;
  ASSERT_TRUE(history.push(8.0, past));  // t0 - 2 s
  LearnedLayout layout;
  int h = 0, w = 0;
  const auto x = build_input(f, 10.0, history, true, layout, h, w);
  ASSERT_EQ(h, 2);
  ASSERT_EQ(w, 2);
  ASSERT_EQ(x.size(), 6u * 4u);
  const auto at = [&](int c, int r, int col) { return x[static_cast<std::size_t>(c) * 4 + r * 2 + col]; };
  EXPECT_FLOAT_EQ(at(0, 0, 0), 2.0f / 16.0f);  // static
  EXPECT_FLOAT_EQ(at(0, 1, 1), 0.0f);          // the observed obstacle is not static
  EXPECT_FLOAT_EQ(at(1, 1, 1), 1.0f / 16.0f);  // observed at t0
  EXPECT_FLOAT_EQ(at(2, 1, 1), 0.0f);          // t0 - 1 s: no frame stored
  EXPECT_FLOAT_EQ(at(3, 1, 1), 1.0f / 16.0f);  // t0 - 2 s
  EXPECT_FLOAT_EQ(at(5, 0, 1), 1.0f);          // observed area
  EXPECT_FLOAT_EQ(at(5, 0, 0), 0.0f);

  // Ablation: past frames are zero.
  const auto no_history = build_input(f, 10.0, history, false, layout, h, w);
  for (int c = 2; c <= 4; ++c)
    for (int i = 0; i < 4; ++i) EXPECT_FLOAT_EQ(no_history[static_cast<std::size_t>(c) * 4 + i], 0.0f);
}

TEST(LearnedInput, HistoryFramesBySimulationTime) {
  FusedMap f = fused_with_observation(4, 4);
  ObservationHistory history(1.0);
  for (int t = 0; t <= 5; ++t) {
    FusedMap frame = f;
    frame.observed_obstacle[static_cast<std::size_t>(t % 4)] = 1;  // marks which frame it is
    history.push(static_cast<double>(t), frame);
    EXPECT_FALSE(history.push(t + 0.5, frame)) << "only one frame per period";
  }
  EXPECT_LE(history.size(), static_cast<std::size_t>(LearnedLayout::kHistory + 1));
  int h = 0, w = 0;
  const auto x = build_input(f, 5.0, history, true, LearnedLayout{}, h, w);
  // t0 - 1 s -> frame t = 4 (cell 0), t0 - 2 s -> t = 3 (cell 3), t0 - 3 s -> t = 2 (cell 2).
  EXPECT_FLOAT_EQ(x[2], 1.0f / 16.0f);
  EXPECT_FLOAT_EQ(x[3], 1.0f / 16.0f);
  EXPECT_FLOAT_EQ(x[4], 1.0f / 16.0f);
  // A time jump backwards (simulation reset) clears the history.
  EXPECT_TRUE(history.push(1.0, f));
  EXPECT_EQ(history.size(), 1u);
}

TEST(LearnedOutput, PriorOutsideObservedAreaAndValidation) {
  Grid g = make_grid(0.4, 0.2, 0.05);  // 8 x 4 fine, 2 x 1 coarse
  std::vector<std::uint8_t> area(g.size(), 0);
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x) area[g.index(x, y)] = 1;
  const auto extra = extra_cost_from_output({30.0f, 2.0f}, 2, 1, g, area, 4, 0.25, 10.0);
  EXPECT_DOUBLE_EQ(extra[g.index(0, 0)], 10.0);  // clamped
  EXPECT_DOUBLE_EQ(extra[g.index(6, 2)], 0.25);  // unobserved: prior
  EXPECT_THROW(extra_cost_from_output({1.0f, std::nanf("")}, 2, 1, g, area, 4, 0.25, 10.0), std::runtime_error);
  EXPECT_THROW(extra_cost_from_output({1.0f, -0.1f}, 2, 1, g, area, 4, 0.25, 10.0), std::runtime_error);
  EXPECT_THROW(extra_cost_from_output({1.0f}, 1, 1, g, area, 4, 0.25, 10.0), std::runtime_error);
}

TEST(LearnedCostMap, KeepsHardMaskAndAdmissibleHeuristic) {
  std::mt19937 rng(3);
  for (int trial = 0; trial < 30; ++trial) {
    FusedMap f = fused_with_observation(40, 40);
    for (auto &v : f.grid.data) v = (rng() % 100 < 15) ? kOccupied : kFree;
    std::vector<double> extra(f.grid.size());
    std::uniform_real_distribution<double> u(0.0, 5.0);
    for (auto &e : extra) e = u(rng);
    CostParams p;
    p.robot_radius = 0.03;
    p.inflation_margin = 0.0;
    const CostMap classical(f, p);
    const CostMap learned(f, p, &extra);
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 40; ++x) {
        ASSERT_EQ(classical.traversable({x, y}), learned.traversable({x, y}));
        if (learned.traversable({x, y})) {
          ASSERT_GE(learned.cost({x, y}), CostMap::kMinCostPerMeter);
        }
      }
    }
    std::uniform_int_distribution<int> coord(0, 39);
    const Cell s{coord(rng), coord(rng)}, t{coord(rng), coord(rng)};
    SearchOptions astar, dijkstra;
    astar.start_escape_radius = dijkstra.start_escape_radius = 0.0;
    dijkstra.use_heuristic = false;
    const auto a = plan_on_grid(learned, s, t, astar);
    const auto d = plan_on_grid(learned, s, t, dijkstra);
    ASSERT_EQ(a.status, d.status);
    if (a.status == PlanStatus::kSuccess) {
      EXPECT_NEAR(a.cost, d.cost, 1e-9);
      EXPECT_LE(octile_heuristic(learned, s, t), d.cost + 1e-12);
    }
  }
  FusedMap f = fused_with_observation(4, 4);
  std::vector<double> bad(f.grid.size(), -1.0);
  EXPECT_THROW(CostMap(f, CostParams{}, &bad), std::invalid_argument);
}

TEST(CostModel, RejectsMissingOrInvalidFile) {
  EXPECT_THROW(CostModel("/nonexistent/planning_cost.onnx", 1), std::runtime_error);
  const std::string not_a_model = ::testing::TempDir() + "/not_a_model.onnx";
  std::ofstream(not_a_model) << "garbage";
  EXPECT_THROW(CostModel(not_a_model, 1), std::runtime_error);
}

TEST(CostModel, ReleasedModelMatchesPyTorchGolden) {
  CostModel model(kModels + "planning_cost.onnx", 1);
  EXPECT_GE(model.info().unobserved_prior, 0.0);
  EXPECT_EQ(model.info().block, 4);
  std::ifstream golden(kModels + "planning_cost_golden.txt");
  ASSERT_TRUE(golden.good());
  int n = 0, c = 0, h = 0, w = 0;
  golden >> n >> c >> h >> w;
  ASSERT_EQ(c, LearnedLayout::kChannels);
  std::vector<float> input(static_cast<std::size_t>(c) * h * w), expected(static_cast<std::size_t>(h) * w);
  for (auto &v : input) golden >> v;
  for (auto &v : expected) golden >> v;
  ASSERT_TRUE(golden.good());
  const auto output = model.run(input, h, w);
  ASSERT_EQ(output.size(), expected.size());
  for (std::size_t i = 0; i < output.size(); ++i) {
    EXPECT_NEAR(output[i], expected[i], 1e-4) << i;
    EXPECT_GE(output[i], 0.0f);
  }
  EXPECT_THROW(model.run(input, h, w + 1), std::runtime_error);
  // Any map size works (fully convolutional): the warehouse map is 100 x 100 blocks.
  EXPECT_EQ(model.run(std::vector<float>(6 * 100 * 103, 0.0f), 100, 103).size(), 100u * 103u);
}

}  // namespace
