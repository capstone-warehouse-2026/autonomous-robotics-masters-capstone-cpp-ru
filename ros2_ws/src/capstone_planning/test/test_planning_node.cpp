#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_planning/planning_node.hpp"

namespace {

class PlanningNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PlanningNodeTest, DefaultsToClassicalMethod) {
  capstone_planning::PlanningNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(PlanningNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_planning::PlanningNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(PlanningNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_planning::PlanningNode node(options), std::invalid_argument);
}

}  // namespace
