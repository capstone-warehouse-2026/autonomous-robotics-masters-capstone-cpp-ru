#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include "capstone_fleet_runtime/robot_navigator.hpp"
#include "capstone_fleet_runtime/scene.hpp"
#include "capstone_fleet_runtime/timed_input.hpp"
#include "capstone_fleet_runtime/fleet_jobs.hpp"

namespace {
using namespace std::chrono_literals;
using capstone_fleet_runtime::TimedInput;
using capstone_fleet_runtime::RobotNavigator;
using capstone_fleet_runtime::FleetJobQueue;
using capstone_mission::JobState;

std::vector<capstone_mission::Job> jobs(double second_release = 0.0) {
  capstone_mission::Job first, second;
  first.id = "first";
  first.pickup = "left";
  first.dropoff = "right";
  first.due_time = 20;
  second = first;
  second.id = "second";
  second.release_time = second_release;
  return {first, second};
}

TEST(FleetQueue, DifferentRobotsCanOwnAndCarryDeliveriesConcurrently) {
  FleetJobQueue queue(jobs());
  queue.release(0);
  queue.assign("first", "robot_1");
  queue.assign("second", "robot_2");
  EXPECT_EQ(queue.find("first").state, JobState::Active);
  EXPECT_EQ(queue.find("second").state, JobState::Active);
  queue.mark_picked("first", "robot_1", 1);
  queue.mark_picked("second", "robot_2", 1);
  EXPECT_EQ(queue.find("first").state, JobState::Picked);
  EXPECT_EQ(queue.find("second").state, JobState::Picked);
  EXPECT_FALSE(queue.all_terminal());
  queue.mark_delivered("first", "robot_1", 2);
  queue.mark_delivered("second", "robot_2", 2);
  EXPECT_TRUE(queue.all_terminal());
}

TEST(FleetQueue, AJobCannotBeAssignedTwiceAndEachRobotHasCapacityOne) {
  FleetJobQueue queue(jobs());
  queue.release(0);
  queue.assign("first", "robot_1");
  EXPECT_THROW(queue.assign("first", "robot_2"), std::invalid_argument);
  EXPECT_THROW(queue.assign("second", "robot_1"), std::invalid_argument);
  EXPECT_EQ(queue.owner("first"), "robot_1");
  EXPECT_TRUE(queue.owner("second").empty());
  EXPECT_EQ(queue.find("second").state, JobState::Released);
  queue.mark_picked("first", "robot_1", 1);
  EXPECT_THROW(queue.assign("second", "robot_1"), std::invalid_argument);
}

TEST(FleetQueue, OnlyOwnerCanPickDeliverOrFailItsJob) {
  FleetJobQueue queue(jobs());
  queue.release(0);
  queue.assign("first", "robot_1");
  EXPECT_THROW(queue.mark_picked("first", "robot_2", 1), std::invalid_argument);
  EXPECT_THROW(queue.mark_failed("first", "robot_2", "cancel"), std::invalid_argument);
  EXPECT_EQ(queue.find("first").state, JobState::Active);
  queue.mark_picked("first", "robot_1", 1);
  EXPECT_THROW(queue.mark_delivered("first", "robot_2", 2), std::invalid_argument);
  EXPECT_THROW(queue.mark_delivered("first", "robot_1", 0.5), std::invalid_argument);
  EXPECT_EQ(queue.find("first").state, JobState::Picked);
  queue.mark_delivered("first", "robot_1", 2);
  EXPECT_EQ(queue.owner("first"), "robot_1");
}

TEST(FleetQueue, ReleasedSecondWaveWaitsUntilRobotHasDeliveredFirstJob) {
  FleetJobQueue queue(jobs(5));
  queue.release(0);
  queue.assign("first", "robot_1");
  EXPECT_THROW(queue.assign("second", "robot_2"), std::invalid_argument);
  queue.mark_picked("first", "robot_1", 1);
  queue.release(5);
  EXPECT_EQ(queue.find("second").state, JobState::Released);
  EXPECT_THROW(queue.assign("second", "robot_1"), std::invalid_argument);
  queue.mark_delivered("first", "robot_1", 6);
  queue.assign("second", "robot_1");
  queue.mark_picked("second", "robot_1", 7);
  queue.mark_delivered("second", "robot_1", 8);
  EXPECT_TRUE(queue.all_terminal());
  EXPECT_EQ(queue.owner("first"), "robot_1");
  EXPECT_EQ(queue.owner("second"), "robot_1");
}

TEST(FleetQueue, InvalidTimesAndUnknownJobsCannotChangeTheQueue) {
  FleetJobQueue queue(jobs());
  EXPECT_THROW(queue.release(-1), std::invalid_argument);
  EXPECT_THROW(queue.assign("absent", "robot_1"), std::invalid_argument);
  EXPECT_THROW(queue.owner("absent"), std::invalid_argument);
  queue.release(0);
  EXPECT_THROW(queue.assign("first", ""), std::invalid_argument);
  EXPECT_EQ(queue.find("first").state, JobState::Released);
}

TEST(FleetQueue, SingleRobotG5RetainsItsOriginalGlobalCapacityInvariant) {
  capstone_mission::JobQueue single(jobs());
  single.release(0);
  single.mark_active("first");
  EXPECT_THROW(single.mark_active("second"), std::invalid_argument);
}

TEST(TimedInput, FutureMessageWaitsForClockAndKeepsOriginalStampAndArrival) {
  TimedInput<int> input;
  const auto wall = TimedInput<int>::Clock::now();
  input.receive(1, 10.0, 10.0, wall);
  input.receive(2, 10.02, 10.0, wall + 10ms);
  ASSERT_TRUE(input.sample());
  EXPECT_EQ(input.sample()->message, 1);
  input.advance(10.02, wall + 30ms);
  EXPECT_EQ(input.sample()->message, 2);
  EXPECT_DOUBLE_EQ(input.sample()->stamp, 10.02);
  EXPECT_EQ(input.sample()->received, wall + 10ms);
}

TEST(TimedInput, RepeatedFuturePacketsCannotExtendOriginalDeadline) {
  TimedInput<int> input;
  const auto wall = TimedInput<int>::Clock::now();
  input.receive(1, 10, 10, wall);
  input.receive(2, 10.02, 10, wall + 10ms);
  input.receive(2, 10.02, 10, wall + 200ms);
  input.advance(10.02, wall + 320ms);
  EXPECT_EQ(input.sample()->message, 1);
  EXPECT_FALSE(input.fresh(10.02, wall + 320ms));
}

TEST(TimedInput, BackwardClockInvalidatesPreviousEpochAndPendingPacket) {
  TimedInput<int> input;
  const auto wall = TimedInput<int>::Clock::now();
  input.receive(1, 10, 10, wall);
  input.receive(2, 10.02, 10, wall);
  input.advance(9.98, wall + 10ms);
  EXPECT_FALSE(input.sample());
  input.receive(3, 9.98, 9.98, wall + 20ms);
  input.advance(10.02, wall + 30ms);
  EXPECT_EQ(input.sample()->message, 3);
}

nav_msgs::msg::OccupancyGrid map() {
  nav_msgs::msg::OccupancyGrid result;
  result.header.frame_id = "map";
  result.info.width = result.info.height = 60;
  result.info.resolution = 0.1F;
  result.info.origin.position.x = result.info.origin.position.y = -2.0;
  result.info.origin.orientation.w = 1;
  result.data.assign(3600, 0);
  return result;
}

TEST(StaticMap, CapsuleRejectsEdgeThatCrossesObstacleBetweenFreeEndpoints) {
  auto occupancy = map();
  occupancy.data[20 * 60 + 24] = 100;
  capstone_fleet::Grid lattice{2, 1, 1.0, 0.0, 0.0, {0, 0}, {}};
  const auto grid = capstone_fleet_runtime::grid_from_map(lattice, occupancy, 0.2);
  EXPECT_TRUE(grid.free({0, 0}));
  EXPECT_TRUE(grid.free({1, 0}));
  EXPECT_FALSE(grid.traversable({0, 0}, {1, 0}));
}

TEST(StaticMap, UnknownCellsBlockFootprintAndRotatedOrNonfiniteMapsAreRejected) {
  auto occupancy = map();
  occupancy.data[20 * 60 + 20] = -1;
  capstone_fleet::Grid lattice{1, 1, 1.0, 0.0, 0.0, {0}, {}};
  EXPECT_FALSE(capstone_fleet_runtime::grid_from_map(lattice, occupancy, 0.2).free({0, 0}));
  occupancy.info.origin.orientation.z = 0.7;
  EXPECT_THROW(capstone_fleet_runtime::grid_from_map(lattice, occupancy, 0.2), std::invalid_argument);
  occupancy.info.origin.orientation.z = 0;
  occupancy.info.origin.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(capstone_fleet_runtime::grid_from_map(lattice, occupancy, 0.2), std::invalid_argument);
}

TEST(Scene, SwapScenarioHasDistinctRobotsAndOppositeDeliveryTargets) {
  const auto scene = capstone_fleet_runtime::load_scene(
      std::string(FLEET_RUNTIME_SOURCE_DIR) + "/scenarios/swap.yaml");
  ASSERT_EQ(scene.robots.size(), 2U);
  EXPECT_NE(scene.robots[0].id, scene.robots[1].id);
  EXPECT_EQ(scene.mission.jobs.size(), 2U);
  EXPECT_EQ(capstone_fleet_runtime::station_cell(scene, "left"), (capstone_fleet::Cell{2, 1}));
  EXPECT_EQ(capstone_fleet_runtime::station_cell(scene, "right"), (capstone_fleet::Cell{8, 1}));
}

TEST(Scene, QueuedScenarioReleasesSecondWaveDuringFirstDeliveries) {
  const auto scene = capstone_fleet_runtime::load_scene(
      std::string(FLEET_RUNTIME_SOURCE_DIR) + "/scenarios/queued.yaml");
  ASSERT_EQ(scene.mission.jobs.size(), 4U);
  EXPECT_DOUBLE_EQ(scene.mission.jobs[0].release_time, 0);
  EXPECT_DOUBLE_EQ(scene.mission.jobs[1].release_time, 0);
  EXPECT_DOUBLE_EQ(scene.mission.jobs[2].release_time, 5);
  EXPECT_DOUBLE_EQ(scene.mission.jobs[3].release_time, 5);
  EXPECT_DOUBLE_EQ(scene.mission.horizon, 480);
}

class RobotActionTest : public ::testing::Test {
 protected:
  using Follow = nav2_msgs::action::FollowPath;
  using Handle = rclcpp_action::ServerGoalHandle<Follow>;
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    client_node_ = std::make_shared<rclcpp::Node>("fleet_action_test_client");
    server_node_ = std::make_shared<rclcpp::Node>("fleet_action_test_server");
    odom_pub_ = server_node_->create_publisher<nav_msgs::msg::Odometry>("/robot_test/state/odom", 5);
    pose_pub_ = server_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot_test/localization/pose", 5);
    server_ = rclcpp_action::create_server<Follow>(server_node_, "/robot_test/control/follow_path",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT :
                           rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }, [this](std::shared_ptr<Handle>) {
          ++cancel_count_;
          return rclcpp_action::CancelResponse::ACCEPT;
        }, [this](std::shared_ptr<Handle> handle) { handle_ = std::move(handle); });
    timer_ = server_node_->create_wall_timer(10ms, [this] { publish_and_finish(); });
    navigator_ = std::make_unique<RobotNavigator>(*client_node_, "robot_test");
    executor_.add_node(client_node_);
    executor_.add_node(server_node_);
    ASSERT_TRUE(until([this] { return navigator_->ready() && navigator_->settled_at(0, 0); }));
  }
  void TearDown() override {
    executor_.remove_node(client_node_);
    executor_.remove_node(server_node_);
    navigator_.reset();
    timer_.reset();
    server_.reset();
    server_node_.reset();
    client_node_.reset();
  }
  void publish_and_finish() {
    if (handle_ && handle_->is_active()) {
      auto result = std::make_shared<Follow::Result>();
      if (handle_->is_canceling()) {
        handle_->canceled(result);
        handle_.reset();
      } else if (complete_) {
        x_ = handle_->get_goal()->path.poses.back().pose.position.x;
        handle_->succeed(result);
        handle_.reset();
      }
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = server_node_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = moving_ ? 0.3 : 0;
    odom_pub_->publish(odom);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = odom.header.stamp;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x_;
    pose.pose.pose.position.y = y_;
    pose.pose.pose.orientation.w = 1;
    pose_pub_->publish(pose);
  }
  template<class Predicate>
  bool until(Predicate predicate, std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      navigator_->update();
      if (predicate()) return true;
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }
  rclcpp::Node::SharedPtr client_node_, server_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<RobotNavigator> navigator_;
  rclcpp_action::Server<Follow>::SharedPtr server_;
  std::shared_ptr<Handle> handle_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool moving_{false}, complete_{true}, reject_{false};
  int cancel_count_{0};
  double x_{0}, y_{0};
};

TEST_F(RobotActionTest, SucceededResultDoesNotOpenBarrierUntilFreshPhysicalStop) {
  moving_ = true;
  navigator_->follow_edge(0, 0, 1, 0);
  ASSERT_TRUE(until([this] { return navigator_->state() == RobotNavigator::ActionState::Succeeded; }));
  EXPECT_FALSE(navigator_->edge_complete());
  EXPECT_FALSE(until([this] { return navigator_->edge_complete(); }, 100ms));
  moving_ = false;
  EXPECT_TRUE(until([this] { return navigator_->edge_complete(); }));
}

TEST_F(RobotActionTest, StaticMapPathPosesUseLatestTransformAfterLocalizationCorrections) {
  complete_ = false;
  navigator_->follow_edge(0, 0, 1, 0);
  ASSERT_TRUE(until([this] { return handle_ != nullptr; }));
  const auto &path = handle_->get_goal()->path;
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_GT(rclcpp::Time(path.header.stamp).nanoseconds(), 0);
  ASSERT_GE(path.poses.size(), 2U);
  for (const auto &pose : path.poses) {
    EXPECT_EQ(pose.header.frame_id, "map");
    EXPECT_EQ(rclcpp::Time(pose.header.stamp).nanoseconds(), 0);
  }
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 1);
  navigator_->cancel();
  ASSERT_TRUE(until([this] { return navigator_->stop_complete(); }));
}

TEST_F(RobotActionTest, RejectedGoalIsTerminalFailure) {
  reject_ = true;
  navigator_->follow_edge(0, 0, 1, 0);
  ASSERT_TRUE(until([this] { return navigator_->terminal(); }));
  EXPECT_EQ(navigator_->state(), RobotNavigator::ActionState::Failed);
  EXPECT_EQ(navigator_->reason(), "goal_rejected");
  EXPECT_FALSE(navigator_->edge_complete());
}

TEST_F(RobotActionTest, CancellationBeforeGoalResponseCancelsLateOwnedGoal) {
  complete_ = false;
  navigator_->follow_edge(0, 0, 1, 0);
  navigator_->cancel();
  ASSERT_TRUE(until([this] { return navigator_->terminal(); }));
  EXPECT_EQ(cancel_count_, 1);
  EXPECT_EQ(navigator_->state(), RobotNavigator::ActionState::Cancelled);
  EXPECT_TRUE(navigator_->stopped());
  EXPECT_FALSE(navigator_->stop_complete());
  EXPECT_FALSE(until([this] { return navigator_->stop_complete(); }, 100ms));
  EXPECT_TRUE(until([this] { return navigator_->stop_complete(); }));
  EXPECT_FALSE(navigator_->edge_complete());
}

TEST_F(RobotActionTest, CancellationNeedsQuietSamplesAfterResultAndAnyLateMotion) {
  complete_ = false;
  navigator_->follow_edge(0, 0, 1, 0);
  ASSERT_TRUE(until([this] { return navigator_->state() == RobotNavigator::ActionState::Active; }));
  ASSERT_TRUE(navigator_->stopped());
  navigator_->cancel();
  ASSERT_TRUE(until([this] { return navigator_->terminal(); }));
  EXPECT_FALSE(navigator_->stop_complete());
  EXPECT_FALSE(until([this] { return navigator_->stop_complete(); }, 100ms));
  // Запоздалая команда после result прерывает даже уже начатое тихое окно.
  moving_ = true;
  ASSERT_TRUE(until([this] { return !navigator_->stopped(); }));
  moving_ = false;
  EXPECT_FALSE(until([this] { return navigator_->stop_complete(); }, 100ms));
  EXPECT_TRUE(until([this] { return navigator_->stop_complete(); }));
}

TEST_F(RobotActionTest, CancellingIdleRobotRequiresFreshQuietWindowAfterRequest) {
  ASSERT_TRUE(navigator_->stop_complete());
  navigator_->cancel();
  EXPECT_FALSE(navigator_->stop_complete());
  EXPECT_FALSE(until([this] { return navigator_->stop_complete(); }, 100ms));
  EXPECT_TRUE(until([this] { return navigator_->stop_complete(); }));
}

TEST_F(RobotActionTest, LeavingReservedEdgeCancelsGoalAndCannotOpenBarrier) {
  complete_ = false;
  navigator_->follow_edge(0, 0, 1, 0);
  ASSERT_TRUE(until([this] { return navigator_->state() == RobotNavigator::ActionState::Active; }));
  x_ = 0.3;
  y_ = 0.2;
  ASSERT_TRUE(until([this] { return navigator_->terminal() && cancel_count_ == 1; }));
  EXPECT_EQ(navigator_->reason(), "left_reserved_corridor");
  EXPECT_FALSE(navigator_->edge_complete());
}

TEST_F(RobotActionTest, MotionAndZeroWithinOneExecutorBatchResetQuietWindow) {
  navigator_->follow_edge(0, 0, 1, 0);
  ASSERT_TRUE(until([this] { return navigator_->state() == RobotNavigator::ActionState::Succeeded; }));
  until([] { return false; }, 100ms);
  timer_->cancel();
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = server_node_->now();
  odom.twist.twist.linear.x = 0.3;
  odom_pub_->publish(odom);
  odom.header.stamp = server_node_->now();
  odom.twist.twist.linear.x = 0;
  odom_pub_->publish(odom);
  // Здесь намеренно обрабатываются callbacks без navigator.update между сообщениями.
  const auto deadline = std::chrono::steady_clock::now() + 30ms;
  while (std::chrono::steady_clock::now() < deadline) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  timer_->reset();
  EXPECT_FALSE(until([this] { return navigator_->edge_complete(); }, 110ms));
  EXPECT_TRUE(until([this] { return navigator_->edge_complete(); }));
}
}  // namespace
