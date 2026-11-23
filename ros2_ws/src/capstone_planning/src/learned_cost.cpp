#include "capstone_planning/learned_cost.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

namespace capstone_planning {

namespace {

Ort::Env &environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "capstone_planning");
  return env;
}

void append_plane(std::vector<float> &tensor, const std::vector<float> &plane) {
  tensor.insert(tensor.end(), plane.begin(), plane.end());
}

}  // namespace

std::vector<float> pool_fraction(const std::vector<std::uint8_t> &fine, int width, int height, int block) {
  if (width <= 0 || height <= 0 || block <= 0 ||
      fine.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
    throw std::invalid_argument("pool_fraction: size mismatch");
  }
  const int cw = (width + block - 1) / block;
  const int ch = (height + block - 1) / block;
  std::vector<float> sum(static_cast<std::size_t>(cw) * ch, 0.0f);
  std::vector<float> count(sum.size(), 0.0f);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t c = static_cast<std::size_t>(y / block) * cw + x / block;
      sum[c] += fine[static_cast<std::size_t>(y) * width + x] ? 1.0f : 0.0f;
      count[c] += 1.0f;
    }
  }
  for (std::size_t i = 0; i < sum.size(); ++i) sum[i] /= count[i];
  return sum;
}

bool ObservationHistory::push(double stamp, const FusedMap &fused) {
  if (!history_.empty()) {
    const ObservationFrame &last = history_.front();
    if (stamp < last.stamp || last.observed_obstacle.size() != fused.observed_obstacle.size()) {
      history_.clear();
    } else if (stamp - last.stamp < period_ - 1e-6) {
      return false;
    }
  }
  history_.push_front({stamp, fused.observed_obstacle, fused.observed_area});
  // Keep one frame more than needed: channel k may skip a frame that is too new.
  while (history_.size() > frames_ + 1) history_.pop_back();
  return true;
}

std::vector<float> build_input(const FusedMap &current, double now, const ObservationHistory &history,
                               bool use_history, const LearnedLayout &layout, int &height, int &width) {
  const Grid &g = current.grid;
  std::vector<std::uint8_t> fine(g.size());
  for (std::size_t i = 0; i < g.size(); ++i) {
    // Static occupancy: obstacles of /map (unknown counts as an obstacle, as in the hard mask).
    const bool observed_only = current.observed_obstacle[i] != 0;
    const std::int8_t v = g.data[i];
    fine[i] = (!observed_only && (is_occupied_value(v) || is_unknown_value(v))) ? 1 : 0;
  }
  std::vector<float> tensor;
  const std::vector<float> static_plane = pool_fraction(fine, g.width, g.height, layout.block);
  height = (g.height + layout.block - 1) / layout.block;
  width = (g.width + layout.block - 1) / layout.block;
  tensor.reserve(static_plane.size() * LearnedLayout::kChannels);
  append_plane(tensor, static_plane);
  append_plane(tensor, pool_fraction(current.observed_obstacle, g.width, g.height, layout.block));
  const std::vector<float> zeros(static_plane.size(), 0.0f);
  for (int k = 1; k < LearnedLayout::kHistory; ++k) {
    const ObservationFrame *frame = nullptr;
    if (use_history) {
      for (const ObservationFrame &f : history.frames()) {
        const double newest = now - (k - 0.5) * layout.history_period;
        const double oldest = now - (k + 0.5) * layout.history_period;
        if (f.stamp <= newest && f.stamp > oldest && f.observed_obstacle.size() == g.size()) {
          frame = &f;
          break;
        }
      }
    }
    append_plane(tensor, frame ? pool_fraction(frame->observed_obstacle, g.width, g.height, layout.block) : zeros);
  }
  append_plane(tensor, pool_fraction(current.observed_area, g.width, g.height, layout.block));
  return tensor;
}

std::vector<double> extra_cost_from_output(const std::vector<float> &output, int coarse_width, int coarse_height,
                                           const Grid &grid, const std::vector<std::uint8_t> &observed_area,
                                           int block, double prior, double max_extra) {
  if (coarse_width != (grid.width + block - 1) / block || coarse_height != (grid.height + block - 1) / block ||
      output.size() != static_cast<std::size_t>(coarse_width) * coarse_height ||
      observed_area.size() != grid.size()) {
    throw std::runtime_error("learned cost: output size does not match the map");
  }
  for (float v : output) {
    if (!std::isfinite(v) || v < 0.0f) {
      throw std::runtime_error("learned cost: non-finite or negative model output");
    }
  }
  std::vector<double> extra(grid.size());
  const double fixed_prior = std::clamp(prior, 0.0, max_extra);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const std::size_t i = grid.index(x, y);
      extra[i] = observed_area[i]
                     ? std::min(static_cast<double>(output[static_cast<std::size_t>(y / block) * coarse_width + x / block]),
                                max_extra)
                     : fixed_prior;
    }
  }
  return extra;
}

struct CostModel::Impl {
  Ort::Session session{nullptr};
  std::string input_name;
  std::string output_name;
};

CostModel::CostModel(const std::string &path, int threads) : impl_(std::make_unique<Impl>()) {
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(std::max(1, threads));
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  try {
    impl_->session = Ort::Session(environment(), path.c_str(), options);
  } catch (const Ort::Exception &error) {
    throw std::runtime_error("cannot load learned cost model '" + path + "': " + error.what());
  }
  Ort::Session &session = impl_->session;
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
    throw std::runtime_error("learned cost model must have one input and one output");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  impl_->input_name = session.GetInputNameAllocated(0, allocator).get();
  impl_->output_name = session.GetOutputNameAllocated(0, allocator).get();
  const auto input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  const auto output_shape = session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (input_shape.size() != 4 || input_shape[1] != LearnedLayout::kChannels || output_shape.size() != 4 ||
      output_shape[1] != 1) {
    throw std::runtime_error("learned cost model must map N x 6 x H x W to N x 1 x H x W");
  }
  const Ort::ModelMetadata metadata = session.GetModelMetadata();
  const auto read = [&](const char *key) -> std::string {
    auto value = metadata.LookupCustomMetadataMapAllocated(key, allocator);
    if (!value) throw std::runtime_error(std::string("learned cost model has no metadata '") + key + "'");
    return value.get();
  };
  try {
    info_.history = read("history") == "true";
    info_.unobserved_prior = std::stod(read("unobserved_prior"));
    info_.max_extra_cost = std::stod(read("max_extra_cost"));
    info_.block = std::stoi(read("block"));
    info_.history_period = std::stod(read("history_period_s"));
    if (std::stoi(read("channels")) != LearnedLayout::kChannels) throw std::runtime_error("channel count mismatch");
  } catch (const std::logic_error &error) {
    throw std::runtime_error(std::string("learned cost model metadata: ") + error.what());
  }
  if (info_.block <= 0 || !std::isfinite(info_.unobserved_prior) || info_.unobserved_prior < 0.0 ||
      !std::isfinite(info_.max_extra_cost) || info_.max_extra_cost <= 0.0 || !(info_.history_period > 0.0)) {
    throw std::runtime_error("learned cost model metadata out of range");
  }
}

CostModel::~CostModel() = default;

std::vector<float> CostModel::run(const std::vector<float> &input, int height, int width) {
  const std::size_t plane = static_cast<std::size_t>(height) * width;
  if (height <= 0 || width <= 0 || input.size() != plane * LearnedLayout::kChannels) {
    throw std::runtime_error("learned cost: input size mismatch");
  }
  const std::int64_t input_shape[4] = {1, LearnedLayout::kChannels, height, width};
  const std::int64_t output_shape[4] = {1, 1, height, width};
  std::vector<float> output(plane);
  const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value in = Ort::Value::CreateTensor<float>(memory, const_cast<float *>(input.data()), input.size(), input_shape, 4);
  Ort::Value out = Ort::Value::CreateTensor<float>(memory, output.data(), output.size(), output_shape, 4);
  const char *in_names[] = {impl_->input_name.c_str()};
  const char *out_names[] = {impl_->output_name.c_str()};
  try {
    impl_->session.Run(Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, &out, 1);
  } catch (const Ort::Exception &error) {
    throw std::runtime_error(std::string("learned cost inference failed: ") + error.what());
  }
  return output;
}

}  // namespace capstone_planning
