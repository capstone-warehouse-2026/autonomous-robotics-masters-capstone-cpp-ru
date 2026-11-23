#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "capstone_integration/world_stub_node.hpp"

namespace {

using namespace std::chrono_literals;

// The placeholder world node must publish the G1 contract and integrate /cmd_vel. The test runs on
// the wall clock (use_sim_time=false), so it does not need a simulator.
TEST(WorldStubNode, PublishesContractAndIntegratesCommands) {
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", false)});
  auto node = std::make_shared<capstone_integration::WorldStubNode>(options);
  auto driver = std::make_shared<rclcpp::Node>("test_driver");

  auto command_pub = driver->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(1).reliable());

  nav_msgs::msg::Odometry last_odometry;
  bool have_odometry = false;
  auto odometry_sub = driver->create_subscription<nav_msgs::msg::Odometry>(
      "/state/odom", rclcpp::QoS(10),
      [&](const nav_msgs::msg::Odometry &message) {
        last_odometry = message;
        have_odometry = true;
      });

  sensor_msgs::msg::LaserScan last_scan;
  bool have_scan = false;
  auto scan_sub = driver->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensors/scan", rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::LaserScan &message) {
        last_scan = message;
        have_scan = true;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(driver);

  for (int i = 0; i < 30; ++i) {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = driver->now();
    command.header.frame_id = "base_link";
    command.twist.linear.x = 0.5;
    command_pub->publish(command);
    executor.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  for (int i = 0; i < 40 && !(have_odometry && have_scan); ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(have_odometry) << "no /state/odom";
  ASSERT_TRUE(have_scan) << "no /sensors/scan";
  EXPECT_EQ(last_odometry.header.frame_id, "odom");
  EXPECT_EQ(last_odometry.child_frame_id, "base_link");
  EXPECT_EQ(last_scan.header.frame_id, "base_link");
  ASSERT_EQ(last_scan.ranges.size(), 360U);
  EXPECT_TRUE(std::isinf(last_scan.ranges.front()));
  EXPECT_GT(node->pose().x, -7.5);  // start x is -8.0, commands moved the robot forward
  EXPECT_GT(node->travelled_distance(), 0.2);

  executor.remove_node(driver);
  executor.remove_node(node);
  rclcpp::shutdown();
}

}  // namespace
