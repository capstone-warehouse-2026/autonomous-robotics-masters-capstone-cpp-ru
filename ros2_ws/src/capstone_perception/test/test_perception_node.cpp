#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include <tf2_ros/static_transform_broadcaster.hpp>

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

TEST_F(PerceptionNodeTest, RejectsInvalidInputsAndPublishesFreshGridInOdom) {
  using namespace std::chrono_literals;
  using sensor_msgs::msg::CameraInfo;
  using sensor_msgs::msg::Image;
  using nav_msgs::msg::OccupancyGrid;

  auto node = std::make_shared<capstone_perception::PerceptionNode>();
  auto source = std::make_shared<rclcpp::Node>("perception_contract_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(source);
  tf2_ros::StaticTransformBroadcaster tf(source);
  geometry_msgs::msg::TransformStamped base;
  base.header.frame_id = "odom";
  base.child_frame_id = "base_link";
  base.transform.rotation.w = 1.0;
  geometry_msgs::msg::TransformStamped camera = base;
  camera.header.frame_id = "base_link";
  camera.child_frame_id = "camera_optical_frame";
  tf.sendTransform({base, camera});

  const rclcpp::SensorDataQoS qos;
  auto rgb_pub = source->create_publisher<Image>("/sensors/rgb", qos);
  auto depth_pub = source->create_publisher<Image>("/sensors/depth", qos);
  auto info_pub = source->create_publisher<CameraInfo>("/sensors/camera_info", qos);
  std::vector<OccupancyGrid> grids;
  auto subscription = source->create_subscription<OccupancyGrid>(
      "/perception/obstacles", 10, [&](OccupancyGrid::ConstSharedPtr grid) { grids.push_back(*grid); });
  const auto spin_for = [&](auto duration) {
    const auto until = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  };
  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < discovery_deadline &&
         (!rgb_pub->get_subscription_count() || !depth_pub->get_subscription_count() ||
          !info_pub->get_subscription_count() || !subscription->get_publisher_count())) {
    spin_for(20ms);
  }
  ASSERT_GT(rgb_pub->get_subscription_count(), 0U);
  ASSERT_GT(depth_pub->get_subscription_count(), 0U);
  ASSERT_GT(info_pub->get_subscription_count(), 0U);
  ASSERT_GT(subscription->get_publisher_count(), 0U);

  const auto send = [&](const rclcpp::Time &stamp, bool matching, bool with_depth) {
    Image rgb, depth;
    CameraInfo info;
    rgb.header.stamp = depth.header.stamp = info.header.stamp = stamp;
    depth.header.frame_id = info.header.frame_id = "camera_optical_frame";
    rgb.header.frame_id = matching ? depth.header.frame_id : "another_camera";
    rgb.width = rgb.height = depth.width = depth.height = info.width = info.height = 2;
    rgb.encoding = "rgb8";
    rgb.step = 6;
    rgb.data.resize(12);
    depth.encoding = "32FC1";
    depth.step = 8;
    depth.data.resize(16);
    info.k[0] = info.k[4] = 1.0;
    rgb_pub->publish(rgb);
    info_pub->publish(info);
    if (with_depth) depth_pub->publish(depth);
    spin_for(150ms);
  };

  send(source->now() - rclcpp::Duration::from_seconds(1), true, true);
  EXPECT_TRUE(grids.empty());
  send(source->now(), false, true);
  EXPECT_TRUE(grids.empty());
  send(source->now(), true, false);
  EXPECT_TRUE(grids.empty());
  auto stamp = source->now();
  for (int attempt = 0; attempt < 10 && grids.empty(); ++attempt) {
    stamp = source->now();
    send(stamp, true, true);
  }
  ASSERT_EQ(grids.size(), 1U);
  EXPECT_EQ(grids.front().header.frame_id, "odom");
  EXPECT_EQ(rclcpp::Time(grids.front().header.stamp), stamp);
  EXPECT_EQ(grids.front().info.width, 200U);
  EXPECT_EQ(grids.front().info.height, 200U);
  EXPECT_DOUBLE_EQ(grids.front().info.origin.position.x, -5.0);
  EXPECT_DOUBLE_EQ(grids.front().info.origin.position.y, -5.0);
}

}  // namespace
