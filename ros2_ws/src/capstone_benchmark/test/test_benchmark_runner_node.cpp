#include <stdexcept>

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

}  // namespace
