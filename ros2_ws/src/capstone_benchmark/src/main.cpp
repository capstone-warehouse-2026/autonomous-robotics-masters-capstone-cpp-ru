#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "capstone_benchmark/benchmark_runner_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capstone_benchmark::BenchmarkRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
