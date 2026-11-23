#pragma once

// Method B of G3: a CNN predicts a non-negative extra cost per metre (delay/risk) from the static map
// and the recent observed occupancy. Network input and output conventions are a literal copy of
// ml/planning/sim.py (observation, pool) and ml/planning/train.py (export); keep them in sync.
//
// The hard collision mask and the search stay those of method A: the extra cost is only added to the
// cost of traversable cells (CostMap), so the model can never open an occupied cell, and the per-metre
// cost keeps its lower bound 1, which keeps the octile heuristic admissible.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "capstone_planning/grid.hpp"
#include "capstone_planning/map_fusion.hpp"

namespace capstone_planning {

/// Network geometry (ml/planning/sim.py): 6 channels on blocks of 4 x 4 fine cells.
struct LearnedLayout {
  static constexpr int kHistory = 4;            // observed occupancy at t0, t0 - 1 s, ...
  static constexpr int kChannels = 2 + kHistory;  // static, history..., observed mask
  int block = 4;
  double history_period = 1.0;  // s
};

/// Mean of `fine` (0/1 per fine cell, width x height) over block x block cells; partial blocks at the
/// border average the existing cells (sim.pool). Output: ceil(height / block) x ceil(width / block).
std::vector<float> pool_fraction(const std::vector<std::uint8_t> &fine, int width, int height, int block);

/// One frame of observed occupancy in the static map geometry.
struct ObservationFrame {
  double stamp = 0.0;  // s
  std::vector<std::uint8_t> observed_obstacle;
  std::vector<std::uint8_t> observed_area;
};

/// Keeps one frame per history period (newest first), in the geometry of the current static map.
class ObservationHistory {
 public:
  explicit ObservationHistory(double period = 1.0, int frames = LearnedLayout::kHistory)
      : period_(period), frames_(frames) {}
  /// Stores the observation if at least one period passed since the last stored one; a time jump
  /// backwards (simulation reset) or a different map size clears the history. Returns true if stored.
  bool push(double stamp, const FusedMap &fused);
  void clear() { history_.clear(); }
  const std::deque<ObservationFrame> &frames() const { return history_; }
  std::size_t size() const { return history_.size(); }

 private:
  double period_;
  std::size_t frames_;
  std::deque<ObservationFrame> history_;
};

/// Network input (kChannels x H x W, row-major, channels first) built exactly as sim.observation:
/// static occupancy (occupied or unknown in /map), the observed obstacles of the newest frames
/// (zeros for missing frames, and for frames t0-1.. when `use_history` is false: ablation) and the
/// currently observed area. `current` is the fused map of the plan request (frame t0, time `now`);
/// channel k >= 1 takes the newest stored frame with a stamp in (now - (k + 0.5) p, now - (k - 0.5) p],
/// zeros if there is none (start-up, dropped frames).
std::vector<float> build_input(const FusedMap &current, double now, const ObservationHistory &history,
                               bool use_history, const LearnedLayout &layout, int &height, int &width);

/// Fine-cell extra cost from the coarse network output: the block value inside the observed area,
/// `prior` outside, clamped to [0, max_extra]. Throws std::runtime_error on a wrong size or a
/// non-finite or negative value (the node then falls back to the classical cost).
std::vector<double> extra_cost_from_output(const std::vector<float> &output, int coarse_width, int coarse_height,
                                           const Grid &grid, const std::vector<std::uint8_t> &observed_area,
                                           int block, double prior, double max_extra);

/// Metadata of the model, stored in the ONNX file (metadata_props) by ml/planning/train.py.
struct ModelInfo {
  bool history = true;
  double unobserved_prior = 0.0;
  double max_extra_cost = 10.0;
  int block = 4;
  double history_period = 1.0;
  std::string sha256;
};

/// The CNN in ONNX Runtime (CPU). Throws std::runtime_error if the file cannot be loaded or its
/// input/output or metadata do not match LearnedLayout.
class CostModel {
 public:
  CostModel(const std::string &path, int threads);
  ~CostModel();
  CostModel(const CostModel &) = delete;
  CostModel &operator=(const CostModel &) = delete;

  const ModelInfo &info() const { return info_; }
  /// Runs the network on kChannels x height x width floats; returns height x width extra costs.
  std::vector<float> run(const std::vector<float> &input, int height, int width);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  ModelInfo info_;
};

}  // namespace capstone_planning
