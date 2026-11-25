#include <csignal>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "capstone_fleet_runtime/fleet_node.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void on_signal(int) { stop_requested = 1; }
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int result = 1;
  try {
    auto node = std::make_shared<capstone_fleet_runtime::FleetNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->finished()) {
      if (stop_requested) node->request_shutdown();
      executor.spin_once(std::chrono::milliseconds(20));
    }
    result = node->finished() ? node->exit_code() : 1;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "Fleet runtime failed: %s\n", error.what());
  }
  rclcpp::shutdown();
  return result;
}
