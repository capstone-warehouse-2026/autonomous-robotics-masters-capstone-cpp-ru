#include <gtest/gtest.h>

#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <thread>

#include "capstone_localization/localization_node.hpp"

namespace {

class LocalizationNodeTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
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

TEST_F(LocalizationNodeTest, PublishesAndRejectsStaleImuThenResetsClock) {
    auto node = std::make_shared<capstone_localization::LocalizationNode>();
    auto io = std::make_shared<rclcpp::Node>("g1_test_io");
    auto wheels = io->create_publisher<nav_msgs::msg::Odometry>("/sensors/wheel_odom",
                                                                rclcpp::SensorDataQoS());
    auto imus =
        io->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", rclcpp::SensorDataQoS());
    nav_msgs::msg::Odometry result;
    size_t received = 0;
    auto sub = io->create_subscription<nav_msgs::msg::Odometry>(
        "/state/odom", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr m) {
            result = *m;
            ++received;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(io);
    auto spin = [&]() {
        for (int i = 0; i < 30; ++i) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };
    spin();
    nav_msgs::msg::Odometry w;
    w.header.frame_id = "odom";
    w.child_frame_id = "base_link";
    w.header.stamp = rclcpp::Time(1000000000LL);
    wheels->publish(w);
    spin();
    ASSERT_GT(received, 0u);
    EXPECT_EQ(result.header.frame_id, "odom");
    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = "imu_link";
    imu.header.stamp = rclcpp::Time(1020000000LL);
    imu.angular_velocity.z = 1;
    imus->publish(imu);
    spin();
    w.header.stamp = rclcpp::Time(1020000000LL);
    wheels->publish(w);
    spin();
    EXPECT_NEAR(2 * std::atan2(result.pose.pose.orientation.z, result.pose.pose.orientation.w), .02,
                1e-8);
    w.header.stamp = rclcpp::Time(1200000000LL);
    wheels->publish(w);
    spin();
    EXPECT_NEAR(result.twist.twist.angular.z, 0, 1e-12);
    EXPECT_NEAR(2 * std::atan2(result.pose.pose.orientation.z, result.pose.pose.orientation.w), .02,
                1e-8);
    w.header.stamp = rclcpp::Time(0LL);
    wheels->publish(w);
    spin();
    EXPECT_NEAR(result.pose.pose.orientation.z, 0, 1e-12);
}
}  // namespace
