#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "capstone_planning/planning_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capstone_planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
