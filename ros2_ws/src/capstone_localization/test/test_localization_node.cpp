#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "capstone_localization/localization_node.hpp"

namespace {

class LocalizationNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(LocalizationNodeTest, DefaultsToClassicalMethod) {
  capstone_localization::LocalizationNode node;
  EXPECT_EQ(node.method(), "classical");
}

TEST_F(LocalizationNodeTest, AcceptsLearnedMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "learned"}});
  capstone_localization::LocalizationNode node(options);
  EXPECT_EQ(node.method(), "learned");
}

TEST_F(LocalizationNodeTest, RejectsUnknownMethod) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"method", "heuristic"}});
  EXPECT_THROW(capstone_localization::LocalizationNode node(options), std::invalid_argument);
}

}  // namespace
