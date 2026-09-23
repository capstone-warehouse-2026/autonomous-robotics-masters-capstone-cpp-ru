#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace capstone_perception {

// G2 skeleton: see projects/02_perception.md and docs/INTERFACES.md.
// The `method` parameter selects the implementation: "classical" or "learned".
class PerceptionNode : public rclcpp::Node {
 public:
  explicit PerceptionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }

 private:
  std::string method_;
};

}  // namespace capstone_perception
