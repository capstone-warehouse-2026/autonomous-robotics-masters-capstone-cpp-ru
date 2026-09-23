#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "capstone_mission/mission_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capstone_mission::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
