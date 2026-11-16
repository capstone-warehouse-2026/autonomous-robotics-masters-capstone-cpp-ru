#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "capstone_control/controller.hpp"
#include "capstone_control/ppo_observation.hpp"

namespace capstone_control {

// The ONNX model of method B in ONNX Runtime (CPU): one observation (float 1 x kObsSize, ppo_observation.hpp)
// -> the deterministic action (float 1 x kActionSize, the mean of the Gaussian policy, clipped to [-1, 1]).
// Exported by ml/control/export_onnx.py together with models/ppo_policy.json (hashes, seeds, validation).
// The same pattern as CnnModel in capstone_perception: buffers and tensors allocated once.
class PpoPolicy {
 public:
  // Throws std::runtime_error when the file cannot be loaded or the model does not have exactly one float
  // input [1, kObsSize] and one float output [1, kActionSize].
  explicit PpoPolicy(const std::string &model_path, int threads = 1);

  // Raw network output for one observation of kObsSize floats; throws std::invalid_argument on a wrong size.
  std::array<float, ppo::kActionSize> run(const std::vector<float> &observation);

 private:
  std::string input_name_;
  std::string output_name_;
  std::vector<float> input_;
  std::vector<float> output_;
  Ort::Session session_{nullptr};
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
};

// Method B of G4 behind the common Controller interface (docs/adr/0004): observation from the ControlInput,
// network, action -> target command; the node's CommandLimiter applies the acceleration limits, as for every
// controller. No valid command (std::nullopt) on a missing path, an exception of the network or a non-finite
// output; the node publishes zero and aborts after three such cycles (NO_VALID_CONTROL).
class PpoController : public Controller {
 public:
  explicit PpoController(std::shared_ptr<PpoPolicy> policy);

  void reset() override {}  // the policy has no state
  std::optional<Command> compute(const ControlInput &input) override;

 private:
  std::shared_ptr<PpoPolicy> policy_;
};

}  // namespace capstone_control
