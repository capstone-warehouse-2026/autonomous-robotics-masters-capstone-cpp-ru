#include <atomic>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "capstone_mission/mission_node.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void on_signal(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char **argv) {
  // Ctrl+C сначала отменяет принадлежащую миссии цель, связь ROS сохраняется до result/timeout.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  auto node = std::make_shared<capstone_mission::MissionNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok()) {
    executor.spin_once(std::chrono::milliseconds(20));
    if (stop_requested) {
      node->request_shutdown();
      if (node->shutdown_ready()) break;
    }
  }
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
