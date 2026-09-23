#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace capstone_mission {

// G5 skeleton: see projects/05_mission.md and docs/INTERFACES.md.
// The `method` parameter selects the implementation: "classical" or "learned".
class MissionNode : public rclcpp::Node {
 public:
  explicit MissionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }

 private:
  std::string method_;
};

}  // namespace capstone_mission
