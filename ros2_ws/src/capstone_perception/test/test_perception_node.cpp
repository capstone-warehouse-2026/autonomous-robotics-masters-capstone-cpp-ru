#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_perception/perception_node.hpp"

namespace {

class PerceptionNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PerceptionNodeTest, DefaultsToClassicalMethod) {
  capstone_perception::PerceptionNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(PerceptionNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_perception::PerceptionNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(PerceptionNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_perception::PerceptionNode node(options), std::invalid_argument);
}

}  // namespace
