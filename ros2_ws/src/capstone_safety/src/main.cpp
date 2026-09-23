#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "capstone_safety/safety_monitor_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capstone_safety::SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
