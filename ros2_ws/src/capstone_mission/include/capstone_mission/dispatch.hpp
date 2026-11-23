#pragma once

#include <array>
#include <string>
#include <vector>
#include "capstone_mission/domain.hpp"

namespace capstone_mission {

// Формат весов открыт: один ReLU-слой, строки матриц соответствуют выходам.
class DqnModel {
 public:
  explicit DqnModel(const std::string &path);
  std::array<float, 5> predict(const Observation &observation) const;
 private:
  std::size_t hidden_size_{0};
  std::vector<float> w1_, b1_, w2_, b2_;
};
struct DispatchDecision {
  int action{4};
  bool fallback{false};
  std::string reason;
};
DispatchDecision choose_action(const std::vector<Job *> &candidates,
                               const std::vector<TravelEstimate> &travel,
                               const DispatchContext &context, const DqnModel *model,
                               bool learned, bool congestion_feature = true);

}  // namespace capstone_mission
