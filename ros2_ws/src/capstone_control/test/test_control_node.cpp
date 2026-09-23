#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_control/control_node.hpp"

namespace {

class ControlNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ControlNodeTest, DefaultsToClassicalMethod) {
  capstone_control::ControlNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(ControlNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_control::ControlNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(ControlNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_control::ControlNode node(options), std::invalid_argument);
}

}  // namespace
