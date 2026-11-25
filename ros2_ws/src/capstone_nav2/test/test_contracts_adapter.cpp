#include <chrono>
#include <limits>
#include <thread>
#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include "capstone_nav2/contracts_adapter.hpp"

namespace {
using namespace std::chrono_literals;
class AdapterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    adapter = std::make_shared<capstone_nav2::ContractsAdapter>();
    driver = std::make_shared<rclcpp::Node>("contracts_test_driver");
    executor.add_node(adapter);
    executor.add_node(driver);
  }
  void spin_for(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }
  std::shared_ptr<capstone_nav2::ContractsAdapter> adapter;
  rclcpp::Node::SharedPtr driver;
  rclcpp::executors::SingleThreadedExecutor executor;
};
TEST_F(AdapterTest, OnlyFreshCommandsAreStampedAndNoCmdVelPublisherExists) {
  auto input = driver->create_publisher<geometry_msgs::msg::Twist>("nav2/cmd_vel", 1);
  int received = 0;
  geometry_msgs::msg::TwistStamped last;
  auto output = driver->create_subscription<geometry_msgs::msg::TwistStamped>(
      "control/cmd_vel_raw", 1, [&](const geometry_msgs::msg::TwistStamped &value) {
        ++received; last = value;
      });
  spin_for(300ms);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  input->publish(command);
  spin_for(100ms);
  ASSERT_EQ(received, 1);
  EXPECT_EQ(last.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(last.twist.linear.x, 0.2);
  spin_for(350ms);
  EXPECT_EQ(received, 1);
  EXPECT_EQ(driver->count_publishers("cmd_vel"), 0U);
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  input->publish(command);
  spin_for(100ms);
  EXPECT_EQ(received, 2);
  EXPECT_DOUBLE_EQ(last.twist.linear.x, 0.0);
}
TEST_F(AdapterTest, ForwardsActualAmclMeasurementWithoutRestamping) {
  auto input = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
  int received = 0;
  geometry_msgs::msg::PoseWithCovarianceStamped last;
  auto output = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", 5, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &value) {
        ++received; last = value;
      });
  spin_for(300ms);
  geometry_msgs::msg::PoseWithCovarianceStamped measurement;
  measurement.header.frame_id = "map";
  measurement.header.stamp = driver->now() - rclcpp::Duration::from_seconds(0.05);
  measurement.pose.pose.position.x = -3;
  measurement.pose.pose.orientation.w = 1;
  input->publish(measurement);
  spin_for(100ms);
  ASSERT_EQ(received, 1);
  EXPECT_EQ(last.header.stamp, measurement.header.stamp);
  spin_for(350ms);
  EXPECT_EQ(received, 1);
  input->publish(measurement);
  spin_for(100ms);
  EXPECT_EQ(received, 1);
  measurement.header.stamp.sec = -1;
  input->publish(measurement);
  spin_for(50ms);
  EXPECT_EQ(received, 1);
}
TEST_F(AdapterTest, AcceptsCovarianceRoundoffButRejectsNegativeVariance) {
  auto input = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
  int received = 0;
  geometry_msgs::msg::PoseWithCovarianceStamped last;
  auto output = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", 5, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &value) {
        ++received; last = value;
      });
  spin_for(300ms);
  geometry_msgs::msg::PoseWithCovarianceStamped measurement;
  measurement.header.frame_id = "map";
  measurement.header.stamp = driver->now();
  measurement.pose.pose.orientation.w = 1.0;
  measurement.pose.covariance[35] = -1.3322676295501869e-15;
  input->publish(measurement);
  spin_for(100ms);
  ASSERT_EQ(received, 1);
  EXPECT_EQ(last.header.stamp, measurement.header.stamp);
  EXPECT_EQ(last.pose.covariance, measurement.pose.covariance);
  measurement.header.stamp = driver->now();
  measurement.pose.covariance[35] = -0.001;
  input->publish(measurement);
  spin_for(100ms);
  EXPECT_EQ(received, 1);
}
TEST_F(AdapterTest, BackwardClockBlocksPreResetLocalizationUntilReinitialized) {
  auto clocks = driver->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
  auto poses = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
  int received = 0;
  auto output = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", 5,
      [&](const geometry_msgs::msg::PoseWithCovarianceStamped &) { ++received; });
  spin_for(300ms);
  rosgraph_msgs::msg::Clock clock;
  clock.clock.sec = 10;
  clocks->publish(clock);
  spin_for(50ms);
  clock.clock.sec = 1;
  clocks->publish(clock);
  spin_for(50ms);
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = driver->now();
  pose.pose.pose.position.x = 4;
  pose.pose.pose.orientation.w = 1;
  poses->publish(pose);
  spin_for(100ms);
  EXPECT_EQ(received, 0);
}
TEST_F(AdapterTest, DelayedTfReleasesPendingScanWithOriginalStamp) {
  auto scans = driver->create_publisher<sensor_msgs::msg::LaserScan>("sensors/scan",
                                                                    rclcpp::SensorDataQoS());
  tf2_ros::StaticTransformBroadcaster broadcaster(driver);
  int received = 0;
  nav_msgs::msg::OccupancyGrid last;
  auto grids = driver->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "perception/obstacles", 1, [&](const nav_msgs::msg::OccupancyGrid &grid) {
        ++received; last = grid;
      });
  spin_for(300ms);
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "test_laser";
  scan.header.stamp = driver->now();
  scan.angle_increment = 0.1;
  scan.range_min = 0.05;
  scan.range_max = 12;
  scan.ranges = {1.0F};
  scans->publish(scan);
  spin_for(30ms);
  EXPECT_EQ(received, 0);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "test_laser";
  transform.header.stamp = driver->now();
  transform.transform.rotation.w = 1;
  broadcaster.sendTransform(transform);
  spin_for(170ms);
  ASSERT_EQ(received, 1);
  EXPECT_EQ(last.header.stamp, scan.header.stamp);
}
TEST_F(AdapterTest, RepeatedScansDoNotStarveWhenTheirTfArrivesLater) {
  auto scans = driver->create_publisher<sensor_msgs::msg::LaserScan>("sensors/scan",
                                                                    rclcpp::SensorDataQoS());
  tf2_ros::TransformBroadcaster broadcaster(driver);
  int received = 0;
  auto grids = driver->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "perception/obstacles", 1, [&](const nav_msgs::msg::OccupancyGrid &) { ++received; });
  spin_for(300ms);
  for (int index = 0; index < 5; ++index) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "delayed_laser";
    scan.header.stamp = driver->now();
    scan.angle_increment = 0.1;
    scan.range_min = 0.05;
    scan.range_max = 12;
    scan.ranges = {1.0F};
    scans->publish(scan);
    spin_for(30ms);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "delayed_laser";
    transform.header.stamp = scan.header.stamp;
    transform.transform.rotation.w = 1;
    broadcaster.sendTransform(transform);
    spin_for(70ms);
    EXPECT_EQ(received, index + 1);
  }
}
TEST_F(AdapterTest, MeasurementsWaitForLateClockWithoutRestamping) {
  executor.remove_node(adapter);
  adapter.reset();
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});
  adapter = std::make_shared<capstone_nav2::ContractsAdapter>(options);
  executor.add_node(adapter);
  auto clocks = driver->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
  auto scans = driver->create_publisher<sensor_msgs::msg::LaserScan>("sensors/scan",
                                                                    rclcpp::SensorDataQoS());
  auto poses = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster(driver);
  int grid_count = 0, pose_count = 0;
  builtin_interfaces::msg::Time grid_stamp, pose_stamp;
  auto grid_sub = driver->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "perception/obstacles", 1, [&](const nav_msgs::msg::OccupancyGrid &grid) {
        ++grid_count; grid_stamp = grid.header.stamp;
      });
  auto pose_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", 5, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
        ++pose_count; pose_stamp = pose.header.stamp;
      });
  spin_for(300ms);
  rosgraph_msgs::msg::Clock clock;
  clock.clock.sec = 10;
  clocks->publish(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "future_laser";
  transform.header.stamp = clock.clock;
  transform.transform.rotation.w = 1;
  broadcaster.sendTransform(transform);
  spin_for(100ms);
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "future_laser";
  scan.header.stamp = clock.clock;
  scan.header.stamp.nanosec = 20000000;
  scan.angle_increment = 0.1;
  scan.range_min = 0.05;
  scan.range_max = 12;
  scan.ranges = {1.0F};
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = scan.header.stamp;
  pose.pose.pose.orientation.w = 1;
  scans->publish(scan);
  poses->publish(pose);
  spin_for(50ms);
  EXPECT_EQ(grid_count, 0);
  EXPECT_EQ(pose_count, 0);
  clock.clock.nanosec = 40000000;
  clocks->publish(clock);
  spin_for(100ms);
  ASSERT_EQ(grid_count, 1);
  ASSERT_EQ(pose_count, 1);
  EXPECT_EQ(grid_stamp, scan.header.stamp);
  EXPECT_EQ(pose_stamp, pose.header.stamp);
}
}  // namespace
