#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace capstone_planning {

// G3 skeleton: see projects/03_planning.md and docs/INTERFACES.md.
// The `method` parameter selects the implementation: "classical" or "learned".
class PlanningNode : public rclcpp::Node {
 public:
  explicit PlanningNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }

 private:
  std::string method_;
};

}  // namespace capstone_planning
