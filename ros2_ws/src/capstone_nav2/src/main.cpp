#include <rclcpp/rclcpp.hpp>
#include "capstone_nav2/contracts_adapter.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capstone_nav2::ContractsAdapter>());
  rclcpp::shutdown();
  return 0;
}
