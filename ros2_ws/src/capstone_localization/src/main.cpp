#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "capstone_localization/localization_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capstone_localization::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
