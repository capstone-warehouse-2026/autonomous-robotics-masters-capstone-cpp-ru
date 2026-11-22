// Tests of the ROS wrapper of the safety monitor.
//  - ScanToPoints: the LaserScan -> points conversion, without a ROS graph.
//  - SafetyMonitorNodeTest: the whole lifecycle node over real topics: a test "driver" node publishes
//    cmd_vel_raw, odom and scan and listens to /cmd_vel and /diagnostics.

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include "capstone_safety/safety_monitor_node.hpp"

namespace {

using namespace std::chrono_literals;
using capstone_safety::Pose2D;
using capstone_safety::scan_to_points;
using diagnostic_msgs::msg::DiagnosticArray;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;

constexpr double kPi = 3.14159265358979323846;

// Scan of 181 rays from -90 to +90 degrees, all equal to range (by default no return).
LaserScan make_scan(float range = std::numeric_limits<float>::infinity(), const std::string &frame = "base_link") {
  LaserScan s;
  s.header.frame_id = frame;
  s.angle_min = static_cast<float>(-kPi / 2);
  s.angle_max = static_cast<float>(kPi / 2);
  s.angle_increment = static_cast<float>(kPi / 180);
  s.range_min = 0.05f;
  s.range_max = 10.0f;
  s.ranges.assign(181, range);
  return s;
}

// ------------------------------------------------------------ scan_to_points

TEST(ScanToPoints, DropsInvalidRanges) {
  LaserScan s = make_scan();
  s.ranges[0] = std::numeric_limits<float>::quiet_NaN();
  s.ranges[1] = 0.01f;   // below range_min
  s.ranges[2] = 20.0f;   // above range_max
  s.ranges[90] = 1.0f;   // the only valid ray, angle 0
  const auto pts = scan_to_points(s, Pose2D{});
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 1.0, 1e-5);
  EXPECT_NEAR(pts[0].y, 0.0, 1e-5);
}

TEST(ScanToPoints, AppliesSensorPose) {
  LaserScan s = make_scan();
  s.ranges[90] = 1.0f;  // ray along the LiDAR axis
  Pose2D lidar;
  lidar.x = 0.2;           // LiDAR 20 cm ahead of the centre
  lidar.yaw = kPi / 2;    // and turned 90 degrees left
  const auto pts = scan_to_points(s, lidar);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 0.2, 1e-5);  // the point is 1 m left of the LiDAR
  EXPECT_NEAR(pts[0].y, 1.0, 1e-5);
}

// ------------------------------------------------------------ node

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

TEST_F(SafetyMonitorNodeTest, RejectsInvalidCoreParams) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"a_brake", 0.0}});
  EXPECT_THROW(capstone_safety::SafetyMonitorNode node(options), std::invalid_argument);
}

// Test bench: the monitor and a driver node in one executor; the monitor is configured and,
// unless told otherwise, activated.
class Bench {
 public:
  explicit Bench(bool activate = true)
      : monitor_(std::make_shared<capstone_safety::SafetyMonitorNode>()),
        driver_(std::make_shared<rclcpp::Node>("safety_test_driver")) {
    cmd_pub_ = driver_->create_publisher<TwistStamped>("/control/cmd_vel_raw", rclcpp::QoS(1).reliable());
    odom_pub_ = driver_->create_publisher<Odometry>("/state/odom", rclcpp::QoS(5).reliable());
    scan_pub_ = driver_->create_publisher<LaserScan>("/sensors/scan", rclcpp::SensorDataQoS());
    out_sub_ = driver_->create_subscription<TwistStamped>(
        "/cmd_vel", rclcpp::QoS(10).reliable(), [this](const TwistStamped &m) { out.push_back(m); });
    diag_sub_ = driver_->create_subscription<DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable(), [this](const DiagnosticArray &m) {
          for (const auto &st : m.status) {
            if (st.name != "capstone_safety/safety_monitor") continue;  // other nodes publish here too
            for (const auto &kv : st.values) {
              if (kv.key == "reason") reason = kv.value;
            }
          }
        });
    exec_.add_node(monitor_->get_node_base_interface());
    exec_.add_node(driver_);

    using lifecycle_msgs::msg::State;
    EXPECT_EQ(monitor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    if (activate) EXPECT_EQ(monitor_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  capstone_safety::SafetyMonitorNode &monitor() { return *monitor_; }

  // Spins the executor for `duration`; every 50 ms publishes the sensors and/or the command if enabled.
  void run(std::chrono::milliseconds duration, bool sensors, std::optional<double> cmd_v,
           float scan_range = std::numeric_limits<float>::infinity(), double odom_v = 0.0,
           const std::string &cmd_frame = "base_link") {
    const auto end = std::chrono::steady_clock::now() + duration;
    auto next_pub = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (std::chrono::steady_clock::now() >= next_pub) {
        next_pub += 50ms;
        const auto stamp = driver_->now();
        if (sensors) {
          Odometry o;
          o.header.stamp = stamp;
          o.header.frame_id = "odom";
          o.child_frame_id = "base_link";
          o.twist.twist.linear.x = odom_v;
          odom_pub_->publish(o);
          LaserScan s = make_scan(scan_range);
          s.header.stamp = stamp;
          scan_pub_->publish(s);
        }
        if (cmd_v) {
          TwistStamped c;
          c.header.stamp = stamp;
          c.header.frame_id = cmd_frame;
          c.twist.linear.x = *cmd_v;
          cmd_pub_->publish(c);
        }
      }
      exec_.spin_some(5ms);
      std::this_thread::sleep_for(2ms);
    }
  }

  std::vector<TwistStamped> out;
  std::string reason;

 private:
  std::shared_ptr<capstone_safety::SafetyMonitorNode> monitor_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Publisher<TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr out_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diag_sub_;
};

TEST_F(SafetyMonitorNodeTest, PublishesZerosWithoutInputs) {
  Bench b;
  b.run(500ms, false, std::nullopt);
  ASSERT_GE(b.out.size(), 3u) << "monitor must publish every tick, even without inputs";
  for (const auto &m : b.out) {
    EXPECT_DOUBLE_EQ(m.twist.linear.x, 0.0);
    EXPECT_EQ(m.header.frame_id, "base_link");
  }
  EXPECT_EQ(b.reason, "NOT_READY");
}

TEST_F(SafetyMonitorNodeTest, PassesFreshCommand) {
  Bench b;
  b.run(800ms, true, 0.3);
  ASSERT_FALSE(b.out.empty());
  EXPECT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.3);
  EXPECT_EQ(b.reason, "OK");
}

TEST_F(SafetyMonitorNodeTest, StopsAndKeepsPublishingZerosWhenControllerIsSilent) {
  Bench b;
  b.run(800ms, true, 0.3);
  ASSERT_FALSE(b.out.empty());
  ASSERT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.3);

  const std::size_t before = b.out.size();
  b.run(600ms, true, std::nullopt);  // sensors alive, controller silent
  ASSERT_GT(b.out.size(), before + 3) << "zeros must keep coming, not a single last Twist";
  EXPECT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.0);
  EXPECT_EQ(b.reason, "STALE_CMD");
}

TEST_F(SafetyMonitorNodeTest, HaltsInFrontOfObstacle) {
  Bench b;
  // Obstacles at 0.5 m on all rays, robot at 0.5 m/s: stopping distance 0.325 m.
  b.run(800ms, true, 0.5, 0.5f, 0.5);
  ASSERT_FALSE(b.out.empty());
  EXPECT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.0);
  EXPECT_EQ(b.reason, "COLLISION_RISK");
}

TEST_F(SafetyMonitorNodeTest, IgnoresCommandInWrongFrame) {
  Bench b;
  b.run(800ms, true, 0.3, std::numeric_limits<float>::infinity(), 0.0, "odom");
  ASSERT_FALSE(b.out.empty());
  EXPECT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.0);
  EXPECT_EQ(b.reason, "NOT_READY");  // the command was rejected, as if it never came
}

// ------------------------------------------------------------ lifecycle

TEST_F(SafetyMonitorNodeTest, AutostartDefaultsToTrue) {
  capstone_safety::SafetyMonitorNode node;
  EXPECT_TRUE(node.autostart());
}

TEST_F(SafetyMonitorNodeTest, ConfigureFailsOnInvalidRate) {
  auto node = std::make_shared<capstone_safety::SafetyMonitorNode>();
  node->set_parameter(rclcpp::Parameter("rate_hz", -1.0));  // broken after construction
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SafetyMonitorNodeTest, PublishesNothingUntilActivated) {
  Bench b(/*activate=*/false);  // configure only
  b.run(500ms, true, 0.3);
  EXPECT_TRUE(b.out.empty()) << "inactive monitor must stay silent; the driver watchdog stops the robot";
}

TEST_F(SafetyMonitorNodeTest, DeactivateSendsStopAndGoesSilent) {
  Bench b;
  b.run(800ms, true, 0.3);
  ASSERT_FALSE(b.out.empty());
  ASSERT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.3);

  const std::size_t before = b.out.size();
  EXPECT_EQ(b.monitor().deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  b.run(500ms, true, 0.3);  // the controller keeps sending commands
  ASSERT_GT(b.out.size(), before) << "the final zero must arrive";
  EXPECT_LE(b.out.size() - before, 2u) << "after deactivate only in-flight messages and one zero are allowed";
  EXPECT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.0);
}

TEST_F(SafetyMonitorNodeTest, ReactivationForgetsOldData) {
  Bench b;
  b.run(800ms, true, 0.3);
  b.monitor().deactivate();
  b.run(300ms, false, std::nullopt);  // no data while inactive
  b.out.clear();
  b.monitor().activate();
  b.run(300ms, false, std::nullopt);  // and none after activation either
  ASSERT_FALSE(b.out.empty());
  EXPECT_DOUBLE_EQ(b.out.back().twist.linear.x, 0.0);
  EXPECT_EQ(b.reason, "NOT_READY");  // the core was reset: the old 0.3 command is not used
}

}  // namespace
