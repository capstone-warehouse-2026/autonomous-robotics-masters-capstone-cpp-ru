#include "capstone_mission/dispatch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace capstone_mission {
namespace {
std::vector<float> weights(const YAML::Node &root, const std::string &key, std::size_t size) {
  auto values = root[key].as<std::vector<float>>();
  if (values.size() != size) throw std::invalid_argument("invalid tensor shape: " + key);
  for (float value : values)
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite weights: " + key);
  return values;
}
}  // namespace
DqnModel::DqnModel(const std::string &path) {
  const auto root = YAML::LoadFile(path);
  if (root["schema_version"].as<int>() != 1 || root["input_size"].as<int>() != 32 ||
      root["output_size"].as<int>() != 5 ||
      root["feature_version"].as<std::string>() != "g5-dispatch-v1")
    throw std::invalid_argument("unsupported DQN model schema");
  const int hidden = root["hidden_size"].as<int>();
  if (hidden < 1 || hidden > 4096) throw std::invalid_argument("invalid hidden layer size");
  hidden_size_ = static_cast<std::size_t>(hidden);
  w1_ = weights(root, "w1", hidden_size_ * 32);
  b1_ = weights(root, "b1", hidden_size_);
  w2_ = weights(root, "w2", hidden_size_ * 5);
  b2_ = weights(root, "b2", 5);
}
std::array<float, 5> DqnModel::predict(const Observation &observation) const {
  for (float value : observation)
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite observation");
  std::vector<float> hidden(hidden_size_);
  for (std::size_t row = 0; row < hidden_size_; ++row) {
    float value = b1_[row];
    for (std::size_t col = 0; col < 32; ++col) value += w1_[row * 32 + col] * observation[col];
    hidden[row] = std::max(0.0F, value);
  }
  std::array<float, 5> output{};
  for (std::size_t row = 0; row < 5; ++row) {
    output[row] = b2_[row];
    for (std::size_t col = 0; col < hidden_size_; ++col)
      output[row] += w2_[row * hidden_size_ + col] * hidden[col];
    if (!std::isfinite(output[row])) throw std::runtime_error("nonfinite DQN output");
  }
  return output;
}
DispatchDecision choose_action(const std::vector<Job *> &candidates,
                               const std::vector<TravelEstimate> &travel,
                               const DispatchContext &context, const DqnModel *model,
                               bool learned, bool congestion_feature) {
  if (context.loaded || candidates.empty()) return {};
  if (!learned) return {classical_action(candidates, travel), false, "classical"};
  try {
    if (!model) throw std::runtime_error("model unavailable");
    const auto output = model->predict(make_observation(candidates, travel, context, congestion_feature));
    const auto mask = make_action_mask(candidates, context.loaded);
    int best = 4;
    float best_value = -std::numeric_limits<float>::infinity();
    for (int action = 0; action < 5; ++action) {
      if (mask[action] && output[action] > best_value) {
        best = action;
        best_value = output[action];
      }
    }
    return {best, false, "dqn"};
  } catch (const std::exception &error) {
    return {classical_action(candidates, travel), true, error.what()};
  }
}
}  // namespace capstone_mission
