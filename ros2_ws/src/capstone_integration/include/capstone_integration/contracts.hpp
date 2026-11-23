#pragma once

#include <string>
#include <vector>

namespace capstone_integration {

// One row of the integration contract (docs/INTERFACES.md). The table lives in contracts.cpp:
// changing the contract is a documented decision (ADR/PR), so a code change is intentional.
struct Contract {
  std::string name;
  std::string type;
  double min_hz{0.0};
  double max_hz{0.0};
  std::string frame;  // empty = frame is not checked
  int min_publishers{1};
  int max_publishers{1};
  bool presence_only{false};  // latched/once-per-episode topics, e.g. /map
  bool required{true};        // missing optional topics are INFO, not FAIL
};

const std::vector<Contract> &contracts();

// Topics that are allowed to exist outside the contract (Webots plumbing, ROS internals).
bool is_allowed_extra_topic(const std::string &name);

}  // namespace capstone_integration
