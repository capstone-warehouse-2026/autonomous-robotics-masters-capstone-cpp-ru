#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace capstone_localization {

// G1 skeleton: see projects/01_localization.md and docs/INTERFACES.md.
// The `method` parameter selects the implementation: "classical" or "learned".
class LocalizationNode : public rclcpp::Node {
 public:
  explicit LocalizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }

 private:
  std::string method_;
};

}  // namespace capstone_localization
