#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include "capstone_safety/safety_monitor_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<capstone_safety::SafetyMonitorNode>();

  // autostart=true: the node configures and activates itself (debugging, standalone runs).
  // autostart=false: G5 drives the transitions through the lifecycle services (/safety_monitor/change_state).
  if (node->autostart()) {
    using lifecycle_msgs::msg::State;
    if (node->configure().id() != State::PRIMARY_STATE_INACTIVE ||
        node->activate().id() != State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "autostart failed: check parameters in the log above");
      rclcpp::shutdown();
      return 1;
    }
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
