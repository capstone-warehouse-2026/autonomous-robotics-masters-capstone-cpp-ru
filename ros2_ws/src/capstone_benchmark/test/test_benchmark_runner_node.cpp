#include <stdexcept>
#include <filesystem>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_benchmark/benchmark_runner_node.hpp"

namespace {

class BenchmarkRunnerNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(BenchmarkRunnerNodeTest, DefaultsToClassicalMethod) {
  capstone_benchmark::BenchmarkRunnerNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(BenchmarkRunnerNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_benchmark::BenchmarkRunnerNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(BenchmarkRunnerNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_benchmark::BenchmarkRunnerNode node(options), std::invalid_argument);
}

TEST_F(BenchmarkRunnerNodeTest, WaitsForStableServiceDiscoveryBeforeReset) {
  using Trigger = std_srvs::srv::Trigger;
  auto server = std::make_shared<rclcpp::Node>("startup_services", "/g5_startup_test");
  int reset_requests = 0;
  auto reset = server->create_service<Trigger>("simulation/reset",
    [&reset_requests](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      ++reset_requests;
      response->success = true;
    });
  auto accept = [](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
    response->success = true;
  };
  auto mission_reset = server->create_service<Trigger>("mission/reset", accept);
  auto mission_start = server->create_service<Trigger>("mission/start", accept);
  auto mission_cancel = server->create_service<Trigger>("mission/cancel", accept);
  const auto directory = std::filesystem::temp_directory_path() /
      ("g5_service_settle_" + std::to_string(getpid()));
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/g5_startup_test"});
  options.parameter_overrides({
    {"run_episode", true}, {"output_directory", directory.string()},
    {"simulation_reset_service", "simulation/reset"}, {"service_settle_time", 0.4},
  });
  auto runner = std::make_shared<capstone_benchmark::BenchmarkRunnerNode>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(runner);
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(200)) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(reset_requests, 0);
  while (reset_requests == 0 && std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(reset_requests, 1);
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(400));
}
TEST_F(BenchmarkRunnerNodeTest, RejectsDiscoveryDelayBeyondStartupDeadline) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"service_settle_time", 20.0}, {"startup_timeout", 10.0}});
  EXPECT_THROW(capstone_benchmark::BenchmarkRunnerNode node(options), std::invalid_argument);
}

}  // namespace
