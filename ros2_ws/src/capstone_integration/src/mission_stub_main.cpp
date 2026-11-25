#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "capstone_integration/mission_stub_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<capstone_integration::MissionStubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
