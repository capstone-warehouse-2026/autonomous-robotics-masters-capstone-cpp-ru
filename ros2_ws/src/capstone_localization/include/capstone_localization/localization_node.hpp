#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace capstone_localization {

// G1 localization: see projects/01_localization.md and docs/INTERFACES.md.
// The `method` parameter selects the implementation: "classical" or "learned".
class LocalizationNode : public rclcpp::Node {
 public:
  explicit LocalizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~LocalizationNode();

  const std::string &method() const { return method_; }

 private:
  struct Impl;
  std::string method_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace capstone_localization
