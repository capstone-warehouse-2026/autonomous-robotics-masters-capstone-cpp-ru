// ROS tests of the planning node: parameters, message conversion and the ComputePathToPose action
// round trip with /map, /perception/obstacles, /localization/pose and TF.

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "capstone_planning/planning_node.hpp"
#include "capstone_planning/scenarios.hpp"

namespace {

using namespace std::chrono_literals;
using capstone_planning::Grid;
using capstone_planning::PlanningNode;
using ComputePath = PlanningNode::ComputePath;
using Result = ComputePath::Result;
namespace scenarios = capstone_planning::scenarios;

nav_msgs::msg::OccupancyGrid to_message(const Grid &grid, const std::string &frame, const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.info.resolution = static_cast<float>(grid.resolution);
  m.info.width = static_cast<uint32_t>(grid.width);
  m.info.height = static_cast<uint32_t>(grid.height);
  m.info.origin.position.x = grid.origin.x;
  m.info.origin.position.y = grid.origin.y;
  m.info.origin.orientation.z = std::sin(0.5 * grid.origin.yaw);
  m.info.origin.orientation.w = std::cos(0.5 * grid.origin.yaw);
  m.data = grid.data;
  return m;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw, const std::string &frame = "map") {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * yaw);
  p.pose.orientation.w = std::cos(0.5 * yaw);
  return p;
}

ComputePath::Goal goal_from(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
  ComputePath::Goal g;
  g.start = start;
  g.goal = goal;
  g.use_start = true;
  return g;
}

class PlanningNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// Planning node + a helper node with publishers and the action client, spun in a background thread.
class Harness {
 public:
  explicit Harness(const std::vector<rclcpp::Parameter> &overrides = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node = std::make_shared<PlanningNode>(options);
    io = std::make_shared<rclcpp::Node>("planning_test_io");
    map_pub = io->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local());
    obstacles_pub = io->create_publisher<nav_msgs::msg::OccupancyGrid>("/perception/obstacles",
                                                                        rclcpp::QoS(1).reliable());
    pose_pub = io->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/pose",
                                                                                   rclcpp::QoS(5).reliable());
    tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(io);
    client = rclcpp_action::create_client<ComputePath>(io, "/planning/compute_path");
    executor.add_node(node);
    executor.add_node(io);
    spinner = std::thread([this] { executor.spin(); });
  }
  ~Harness() {
    executor.cancel();
    spinner.join();
  }

  void publish_map(const Grid &grid) {
    map_pub->publish(to_message(grid, "map", io->now()));
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!node->has_map() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(10ms);
    ASSERT_TRUE(node->has_map());
  }

  void publish_map_from_odom(const capstone_planning::Pose2D &t) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = io->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = t.x;
    tf.transform.translation.y = t.y;
    tf.transform.rotation.z = std::sin(0.5 * t.yaw);
    tf.transform.rotation.w = std::cos(0.5 * t.yaw);
    tf_broadcaster->sendTransform(tf);
  }

  rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult send(const ComputePath::Goal &goal) {
    EXPECT_TRUE(client->wait_for_action_server(5s));
    auto handle_future = client->async_send_goal(goal);
    EXPECT_EQ(handle_future.wait_for(5s), std::future_status::ready);
    auto handle = handle_future.get();
    EXPECT_TRUE(handle != nullptr);
    auto result_future = client->async_get_result(handle);
    EXPECT_EQ(result_future.wait_for(5s), std::future_status::ready);
    return result_future.get();
  }

  std::shared_ptr<PlanningNode> node;
  std::shared_ptr<rclcpp::Node> io;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacles_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
  rclcpp_action::Client<ComputePath>::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
};

const std::vector<rclcpp::Parameter> kNoPerception{rclcpp::Parameter("require_obstacles", false)};

// --- parameters and conversion --------------------------------------------------------------------

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

TEST_F(PlanningNodeTest, RejectsInvalidGeometry) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"robot_radius", -0.1}});
  EXPECT_THROW(capstone_planning::PlanningNode node(options), std::invalid_argument);
}

TEST_F(PlanningNodeTest, GridFromMessage) {
  Grid grid = scenarios::make_grid(1.0, 0.5, 0.05);
  grid.origin = {1.0, 2.0, 0.5};
  auto message = to_message(grid, "map", rclcpp::Time(1, 0));
  const Grid back = capstone_planning::grid_from_message(message);
  EXPECT_EQ(back.width, 20);
  EXPECT_EQ(back.height, 10);
  EXPECT_NEAR(back.origin.yaw, 0.5, 1e-6);
  message.data.pop_back();
  EXPECT_THROW(capstone_planning::grid_from_message(message), std::invalid_argument);
  message = to_message(grid, "map", rclcpp::Time(1, 0));
  message.info.resolution = std::nanf("");
  EXPECT_THROW(capstone_planning::grid_from_message(message), std::invalid_argument);
}

// --- action ---------------------------------------------------------------------------------------

TEST_F(PlanningNodeTest, ReturnsPathInMapFrame) {
  Harness h(kNoPerception);
  h.publish_map(scenarios::two_routes(false, false).static_map);
  const auto r = h.send(goal_from(pose(1.0, 3.0, 0.0), pose(9.0, 3.0, 1.0)));
  ASSERT_EQ(r.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(r.result->error_code, Result::NONE);
  EXPECT_EQ(r.result->path.header.frame_id, "map");
  ASSERT_GE(r.result->path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(r.result->path.poses.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(r.result->path.poses.back().pose.position.x, 9.0);
  EXPECT_NEAR(r.result->path.poses.back().pose.orientation.z, std::sin(0.5), 1e-9);
  for (std::size_t i = 1; i < r.result->path.poses.size(); ++i) {
    const auto &a = r.result->path.poses[i - 1].pose.position;
    const auto &b = r.result->path.poses[i].pose.position;
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), 1.0);  // G4 max_path_gap
  }
  EXPECT_LT(rclcpp::Duration(r.result->planning_time).seconds(), 0.2);
}

TEST_F(PlanningNodeTest, ExplicitFailuresAreAborted) {
  Harness h(kNoPerception);
  // No map yet.
  auto r = h.send(goal_from(pose(1.0, 2.0, 0.0), pose(6.0, 2.0, 0.0)));
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, Result::UNKNOWN);
  EXPECT_TRUE(r.result->path.poses.empty());

  h.publish_map(scenarios::unreachable_goal().static_map);
  r = h.send(goal_from(pose(1.0, 2.0, 0.0), pose(6.0, 2.0, 0.0)));  // goal inside a closed box
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, Result::NO_VALID_PATH);
  EXPECT_TRUE(r.result->path.poses.empty());

  r = h.send(goal_from(pose(1.0, 2.0, 0.0), pose(5.05, 2.0, 0.0)));  // goal in the box wall
  EXPECT_EQ(r.result->error_code, Result::GOAL_OCCUPIED);
  r = h.send(goal_from(pose(1.0, 2.0, 0.0), pose(20.0, 2.0, 0.0)));
  EXPECT_EQ(r.result->error_code, Result::GOAL_OUTSIDE_MAP);
  r = h.send(goal_from(pose(-3.0, 2.0, 0.0), pose(2.0, 2.0, 0.0)));
  EXPECT_EQ(r.result->error_code, Result::START_OUTSIDE_MAP);
  r = h.send(goal_from(pose(1.0, 2.0, 0.0), pose(2.0, 2.0, 0.0, "nowhere")));
  EXPECT_EQ(r.result->error_code, Result::TF_ERROR);
  auto bad = pose(2.0, 2.0, 0.0);
  bad.pose.position.x = std::nan("");
  r = h.send(goal_from(pose(1.0, 2.0, 0.0), bad));
  EXPECT_EQ(r.result->error_code, Result::TF_ERROR);
}

TEST_F(PlanningNodeTest, RequiresFreshObstaclesByDefault) {
  Harness h;
  h.publish_map(scenarios::two_routes(false, false).static_map);
  const auto r = h.send(goal_from(pose(1.0, 3.0, 0.0), pose(9.0, 3.0, 0.0)));
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(r.result->error_msg.find("perception"), std::string::npos) << r.result->error_msg;
}

TEST_F(PlanningNodeTest, ObstaclesInOdomBlockCorridorThroughTf) {
  Harness h;
  const auto s = scenarios::two_routes(true, false);
  h.publish_map(s.static_map);
  h.publish_map_from_odom(s.map_from_local);
  // Keep the local grid fresh while planning: perception publishes at 10 Hz.
  std::atomic<bool> running{true};
  std::thread perception([&] {
    while (running) {
      h.obstacles_pub->publish(to_message(s.local, "odom", h.io->now()));
      std::this_thread::sleep_for(50ms);
    }
  });
  rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult r;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (true) {  // the first requests may come before the grid or the static TF arrive
    r = h.send(goal_from(pose(1.0, 3.0, 0.0), pose(9.0, 3.0, 0.0)));
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED || std::chrono::steady_clock::now() > deadline) break;
    std::this_thread::sleep_for(50ms);
  }
  running = false;
  perception.join();
  ASSERT_EQ(r.code, rclcpp_action::ResultCode::SUCCEEDED) << r.result->error_msg;
  bool through_b = false;
  for (const auto &p : r.result->path.poses) {
    through_b = through_b || (std::abs(p.pose.position.x - 5.0) < 0.1 && p.pose.position.y > 4.4);
  }
  EXPECT_TRUE(through_b) << "gap A is blocked by the observed obstacle";
}

TEST_F(PlanningNodeTest, UsesFreshLocalizationPose) {
  Harness h(kNoPerception);
  h.publish_map(scenarios::two_routes(false, false).static_map);
  ComputePath::Goal goal;
  goal.goal = pose(9.0, 3.0, 0.0);
  goal.use_start = false;

  geometry_msgs::msg::PoseWithCovarianceStamped stale;
  stale.header.frame_id = "map";
  stale.header.stamp = h.io->now() - rclcpp::Duration::from_seconds(5.0);
  stale.pose.pose = pose(1.0, 3.0, 0.0).pose;
  h.pose_pub->publish(stale);
  std::this_thread::sleep_for(200ms);
  auto r = h.send(goal);
  EXPECT_EQ(r.result->error_code, Result::TF_ERROR);

  Result direct;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  do {
    auto fresh = stale;
    fresh.header.stamp = h.io->now();
    fresh.pose.pose = pose(2.0, 2.5, 0.0).pose;
    h.pose_pub->publish(fresh);
    std::this_thread::sleep_for(20ms);
    direct = h.node->plan(goal);
  } while (direct.error_code != Result::NONE && std::chrono::steady_clock::now() < deadline);
  ASSERT_EQ(direct.error_code, Result::NONE) << direct.error_msg;
  EXPECT_NEAR(direct.path.poses.front().pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(direct.path.poses.front().pose.position.y, 2.5, 1e-9);
}

TEST_F(PlanningNodeTest, CancelAndTimeoutStopTheSearch) {
  // Large map with an unreachable goal: the search would expand the whole map.
  Grid grid = scenarios::make_grid(30.0, 30.0, 0.05);
  scenarios::fill_rect(grid, 20.0, 0.0, 20.1, 30.0);
  {
    Harness h(kNoPerception);
    h.publish_map(grid);
    const auto goal = goal_from(pose(1.0, 1.0, 0.0), pose(25.0, 25.0, 0.0));
    const auto canceled = h.node->plan(goal, [] { return true; });
    EXPECT_EQ(canceled.error_code, Result::UNKNOWN);
    EXPECT_EQ(canceled.error_msg, "canceled");
  }
  Harness h({rclcpp::Parameter("require_obstacles", false), rclcpp::Parameter("planning_timeout", 1e-6)});
  h.publish_map(grid);
  const auto r = h.send(goal_from(pose(1.0, 1.0, 0.0), pose(25.0, 25.0, 0.0)));
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, Result::TIMEOUT);
}

}  // namespace
