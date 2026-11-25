#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "capstone_integration/world_stub_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<capstone_integration::WorldStubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
