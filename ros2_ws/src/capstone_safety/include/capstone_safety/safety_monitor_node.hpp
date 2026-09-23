#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace capstone_safety {

// G4 skeleton: see projects/04_control.md and docs/INTERFACES.md.
// The `method` parameter selects the implementation: "classical" or "learned".
class SafetyMonitorNode : public rclcpp::Node {
 public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }

 private:
  std::string method_;
};

}  // namespace capstone_safety
