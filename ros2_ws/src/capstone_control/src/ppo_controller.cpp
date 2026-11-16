#include "capstone_control/ppo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace capstone_control {

namespace {

// One ONNX Runtime environment per process, shared by all sessions (as capstone_perception).
Ort::Env &environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "capstone_control");
  return env;
}

std::string shape_text(const std::vector<std::int64_t> &shape) {
  std::string text;
  for (const auto dimension : shape) {
    text += (text.empty() ? "" : "x") + std::to_string(dimension);
  }
  return text;
}

}  // namespace

PpoPolicy::PpoPolicy(const std::string &model_path, int threads) {
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(threads);
  options.SetInterOpNumThreads(1);
  try {
    session_ = Ort::Session(environment(), model_path.c_str(), options);
  } catch (const Ort::Exception &error) {
    throw std::runtime_error("cannot load the model " + model_path + ": " + error.what());
  }
  if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
    throw std::runtime_error(model_path + ": the model must have one input and one output");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_.GetInputNameAllocated(0, allocator).get();
  output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
  const Ort::TypeInfo input_type = session_.GetInputTypeInfo(0);
  const Ort::TypeInfo output_type = session_.GetOutputTypeInfo(0);
  const auto input_info = input_type.GetTensorTypeAndShapeInfo();
  const auto output_info = output_type.GetTensorTypeAndShapeInfo();
  const std::vector<std::int64_t> input_shape = input_info.GetShape();
  const std::vector<std::int64_t> output_shape = output_info.GetShape();
  const bool shapes_valid = input_shape.size() == 2 && input_shape[0] == 1 &&
                            input_shape[1] == ppo::kObsSize && output_shape.size() == 2 &&
                            output_shape[0] == 1 && output_shape[1] == ppo::kActionSize;
  if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || !shapes_valid) {
    throw std::runtime_error(model_path + ": expected float input 1x" +
                             std::to_string(ppo::kObsSize) + " and output 1x" +
                             std::to_string(ppo::kActionSize) + ", got " +
                             shape_text(input_shape) + " -> " + shape_text(output_shape));
  }

  input_.assign(ppo::kObsSize, 0.0f);
  output_.assign(ppo::kActionSize, 0.0f);
  const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  const std::array<std::int64_t, 2> input_dims{1, ppo::kObsSize};
  const std::array<std::int64_t, 2> output_dims{1, ppo::kActionSize};
  input_tensor_ = Ort::Value::CreateTensor<float>(memory, input_.data(), input_.size(), input_dims.data(),
                                                  input_dims.size());
  output_tensor_ = Ort::Value::CreateTensor<float>(memory, output_.data(), output_.size(),
                                                   output_dims.data(), output_dims.size());
}

std::array<float, ppo::kActionSize> PpoPolicy::run(const std::vector<float> &observation) {
  if (observation.size() != static_cast<std::size_t>(ppo::kObsSize)) {
    throw std::invalid_argument("the observation has " + std::to_string(observation.size()) +
                                " values, expected " + std::to_string(ppo::kObsSize));
  }
  std::copy(observation.begin(), observation.end(), input_.begin());
  const char *input_names[] = {input_name_.c_str()};
  const char *output_names[] = {output_name_.c_str()};
  session_.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);
  std::array<float, ppo::kActionSize> action{};
  std::copy(output_.begin(), output_.end(), action.begin());
  return action;
}

PpoController::PpoController(std::shared_ptr<PpoPolicy> policy) : policy_(std::move(policy)) {
  if (!policy_) {
    throw std::invalid_argument("PpoController requires a non-null PpoPolicy");
  }
}

std::optional<Command> PpoController::compute(const ControlInput &input) {
  if (input.path == nullptr || input.path->empty()) {
    return std::nullopt;
  }
  try {
    const auto action = policy_->run(ppo::build_observation(input));
    if (!std::isfinite(action[0]) || !std::isfinite(action[1])) {
      return std::nullopt;
    }
    return ppo::action_to_target(action[0], action[1]);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace capstone_control