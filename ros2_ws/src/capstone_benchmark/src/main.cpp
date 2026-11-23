#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "capstone_benchmark/benchmark_runner_node.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<capstone_benchmark::BenchmarkRunnerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->finished()) executor.spin_once(std::chrono::milliseconds(100));
  const int code = node->exit_code();
  rclcpp::shutdown();
  return code;
}
