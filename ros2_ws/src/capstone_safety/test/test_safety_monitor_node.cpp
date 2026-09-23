#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_safety/safety_monitor_node.hpp"

namespace {

class SafetyMonitorNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(SafetyMonitorNodeTest, DefaultsToClassicalMethod) {
  capstone_safety::SafetyMonitorNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(SafetyMonitorNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_safety::SafetyMonitorNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(SafetyMonitorNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_safety::SafetyMonitorNode node(options), std::invalid_argument);
}

}  // namespace
