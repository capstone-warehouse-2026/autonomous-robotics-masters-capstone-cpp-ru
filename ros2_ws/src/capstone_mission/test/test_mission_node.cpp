#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_mission/mission_node.hpp"

namespace {

class MissionNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(MissionNodeTest, DefaultsToClassicalMethod) {
  capstone_mission::MissionNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(MissionNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_mission::MissionNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(MissionNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_mission::MissionNode node(options), std::invalid_argument);
}

}  // namespace
