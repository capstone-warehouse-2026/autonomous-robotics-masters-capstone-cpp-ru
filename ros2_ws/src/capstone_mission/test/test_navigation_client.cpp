#include <chrono>
#include <memory>
#include <thread>
#include <fstream>
#include <filesystem>
#include <unistd.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include "capstone_mission/mission_node.hpp"

#include <gtest/gtest.h>
#include "capstone_mission/navigation_client.hpp"

namespace {
using namespace std::chrono_literals;
using Plan = nav2_msgs::action::ComputePathToPose;
using Follow = nav2_msgs::action::FollowPath;
using PlanHandle = rclcpp_action::ServerGoalHandle<Plan>;
using FollowHandle = rclcpp_action::ServerGoalHandle<Follow>;
using Client = capstone_mission::NavigationClient;

class NavigationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("navigation_test_client");
    server_ = std::make_shared<rclcpp::Node>("navigation_test_server");
    odom_pub_ = server_->create_publisher<nav_msgs::msg::Odometry>("state/odom", 5);
    planner_ = rclcpp_action::create_server<Plan>(server_, "planning/compute_path",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Plan::Goal>) {
          ++plan_count_;
          return (reject_plan_ || plan_count_ <= reject_first_) ? rclcpp_action::GoalResponse::REJECT :
                                rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<PlanHandle>) {
          ++cancel_count_;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<PlanHandle> handle) { plan_ = handle; });
    controller_ = rclcpp_action::create_server<Follow>(server_, "control/follow_path",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal>) {
          ++follow_count_;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<FollowHandle>) {
          ++cancel_count_;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<FollowHandle> handle) { follow_ = handle; });
    pose_pub_ = server_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "localization/pose", 5);
    clock_pub_ = server_->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    status_sub_ = server_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "mission/status", 10, [this](const diagnostic_msgs::msg::DiagnosticArray &message) {
          if (message.status.empty()) return;
          mission_status_ = message.status.front().message;
          for (const auto &value : message.status.front().values) {
            if (value.key == "delivered") delivered_ = std::stoi(value.value);
            if (value.key == "episode_id") episode_id_ = value.value;
          }
        });
    timer_ = server_->create_wall_timer(10ms, [this] { publish_and_finish(); });
    Client::Limits limits;
    limits.planning_timeout = 0.2;
    limits.stop_timeout = 0.25;
    limits.navigation_timeout = 1.0;
    client_ = std::make_unique<Client>(*node_, limits,
        [](const std::string &, const std::string &) {});
    executor_.add_node(node_);
    executor_.add_node(server_);
    ASSERT_TRUE(until([this] { return client_->ready(); }, 2s));
  }

  void TearDown() override {
    if (mission_) {
      executor_.remove_node(mission_);
      mission_.reset();
    }
    executor_.remove_node(server_);
    executor_.remove_node(node_);
    client_.reset();
    timer_.reset();
    planner_.reset();
    controller_.reset();
    node_.reset();
    server_.reset();
    if (!scenario_path_.empty()) std::filesystem::remove(scenario_path_);
  }

  void publish_and_finish() {
    if (sim_clock_) {
      if (!freeze_) sim_time_ += 0.01;
      rosgraph_msgs::msg::Clock clock;
      clock.clock = rclcpp::Time(static_cast<int64_t>(sim_time_ * 1e9));
      clock_pub_->publish(clock);
    }
    if (publish_odom_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = sim_clock_ ? rclcpp::Time(static_cast<int64_t>(sim_time_ * 1e9)) :
                                      server_->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = moving_ ? 0.3 : 0.0;
      odom_pub_->publish(odom);
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header = odom.header;
      pose.header.frame_id = "map";
      pose.pose = odom.pose;
      pose_pub_->publish(pose);
    }
    if (plan_ && plan_->is_active()) {
      auto result = std::make_shared<Plan::Result>();
      if (plan_->is_canceling()) {
        plan_->canceled(result);
        plan_.reset();
      } else if (!hold_plan_) {
        result->path.header.frame_id = "map";
        result->path.poses.push_back(plan_->get_goal()->goal);
        if (bad_path_) result->path.poses.front().pose.orientation.w = 0.0;
        plan_->succeed(result);
        plan_.reset();
      }
    }
    if (follow_ && follow_->is_active()) {
      auto result = std::make_shared<Follow::Result>();
      if (follow_->is_canceling() && !ignore_cancel_) {
        follow_->canceled(result);
        follow_.reset();
      } else if (!hold_follow_ && !follow_->is_canceling()) {
        if (follow_count_ == fail_follow_number_) follow_->abort(result);
        else follow_->succeed(result);
        follow_.reset();
      }
    }
  }

  template<class Predicate>
  bool until(Predicate predicate, std::chrono::milliseconds timeout = 1500ms) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      client_->poll();
      if (predicate()) return true;
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }

  void start() {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "map";
    target.header.stamp = node_->now();
    target.pose.orientation.w = 1.0;
    client_->start(target);
  }

  void start_mission(int jobs = 1, bool simulation_clock = false, bool autostart = true) {
    sim_clock_ = simulation_clock;
    scenario_path_ = "/tmp/g5_ros_test_" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(scenario_path_);
    file << "episode_id: test\ninitial_station: home\nhorizon: 30\nstations:\n"
         << "  - {id: home, x: 0, y: 0}\n  - {id: destination, x: 0, y: 0}\njobs:\n";
    for (int i = 0; i < jobs; ++i) {
      file << "  - {id: job_" << i << ", pickup: home, dropoff: destination, release_time: 0, "
           << "due_time: 20, pickup_service: 0.01, dropoff_service: 0.01}\n";
    }
    file.close();
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"manifest_path", scenario_path_}, {"require_safety", false},
        {"event_log", ""}, {"clock_timeout", 0.2}, {"stop_timeout", 0.3},
        {"use_sim_time", simulation_clock}, {"autostart", autostart},
        {"bt_xml", std::string(CAPSTONE_MISSION_SOURCE_DIR) + "/behavior_trees/delivery.xml"}});
    mission_ = std::make_shared<capstone_mission::MissionNode>(options);
    executor_.add_node(mission_);
  }

  std::shared_ptr<capstone_mission::MissionNode> mission_;
  std::string scenario_path_, mission_status_, episode_id_;
  int delivered_{0}, reject_first_{0}, fail_follow_number_{-1};
  bool sim_clock_{false}, freeze_{false};
  double sim_time_{10.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;
  rclcpp::Node::SharedPtr node_, server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_action::Server<Plan>::SharedPtr planner_;
  rclcpp_action::Server<Follow>::SharedPtr controller_;
  std::shared_ptr<PlanHandle> plan_;
  std::shared_ptr<FollowHandle> follow_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Client> client_;
  int plan_count_{0}, follow_count_{0}, cancel_count_{0};
  bool reject_plan_{false}, hold_plan_{false}, hold_follow_{false};
  bool moving_{false}, ignore_cancel_{false}, publish_odom_{true}, bad_path_{false};
};

TEST_F(NavigationTest, SuccessRequiresNewStoppedOdometry) {
  start();
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Succeeded; }));
  EXPECT_TRUE(client_->safe_to_retry());
  EXPECT_EQ(plan_count_, 1);
  EXPECT_EQ(follow_count_, 1);
}

TEST_F(NavigationTest, RejectDoesNotSendControllerGoal) {
  reject_plan_ = true;
  start();
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(client_->reason(), "planning_rejected");
  EXPECT_TRUE(client_->safe_to_retry());
  EXPECT_EQ(follow_count_, 0);
}

TEST_F(NavigationTest, InvalidQuaternionDoesNotReachController) {
  bad_path_ = true;
  start();
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(follow_count_, 0);
  EXPECT_EQ(client_->reason(), "planning_failed");
}

TEST_F(NavigationTest, CancellationWaitsForActionResultAndStop) {
  hold_follow_ = true;
  start();
  ASSERT_TRUE(until([this] { return follow_count_ > 0; }));
  client_->cancel("operator_cancel");
  EXPECT_EQ(client_->state(), Client::State::Stopping);
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(client_->reason(), "operator_cancel");
  EXPECT_TRUE(client_->safe_to_retry());
  EXPECT_EQ(cancel_count_, 1);
}

TEST_F(NavigationTest, CancelBeforeGoalResponseCancelsLateAcceptedGoal) {
  hold_plan_ = true;
  start();
  client_->cancel("cancel_before_response");
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_TRUE(client_->safe_to_retry());
  EXPECT_EQ(cancel_count_, 1);
  EXPECT_EQ(follow_count_, 0);
}

TEST_F(NavigationTest, GoalAcceptedAfterCancelDeadlineIsStillCancelled) {
  hold_plan_ = true;
  start();
  client_->cancel("cancel_before_response");
  // Сервер ещё не обрабатывал запрос, а клиент уже исчерпал срок подтверждения.
  std::this_thread::sleep_for(300ms);
  client_->poll();
  ASSERT_EQ(client_->state(), Client::State::Failed);
  EXPECT_FALSE(client_->safe_to_retry());
  ASSERT_TRUE(until([this] { return cancel_count_ == 1; }));
  EXPECT_EQ(follow_count_, 0);
  EXPECT_THROW(start(), std::logic_error);
}

TEST_F(NavigationTest, AcceptedCancelWithoutResultIsFault) {
  hold_follow_ = true;
  ignore_cancel_ = true;
  start();
  ASSERT_TRUE(until([this] { return follow_count_ > 0; }));
  client_->cancel("cancel");
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(client_->reason(), "action_result_unknown");
  EXPECT_FALSE(client_->safe_to_retry());
}

TEST_F(NavigationTest, SuccessWhileStillMovingIsFault) {
  moving_ = true;
  start();
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(client_->reason(), "failure_to_stop");
  EXPECT_FALSE(client_->safe_to_retry());
}

TEST_F(NavigationTest, PlanningTimeoutCancelsOwnedGoal) {
  hold_plan_ = true;
  start();
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(client_->reason(), "planning_timeout");
  EXPECT_TRUE(client_->safe_to_retry());
  EXPECT_EQ(cancel_count_, 1);
  EXPECT_EQ(follow_count_, 0);
}

TEST_F(NavigationTest, StaleOdometryCancelsMovement) {
  hold_follow_ = true;
  start();
  ASSERT_TRUE(until([this] { return follow_count_ > 0; }));
  publish_odom_ = false;
  ASSERT_TRUE(until([this] { return client_->state() == Client::State::Failed; }));
  EXPECT_EQ(client_->reason(), "failure_to_stop");
  EXPECT_FALSE(client_->safe_to_retry());
  EXPECT_EQ(cancel_count_, 1);
}
TEST_F(NavigationTest, MissionCompletesPickupBeforeDelivery) {
  start_mission();
  ASSERT_TRUE(until([this] { return mission_status_ == "completed"; }, 2500ms));
  EXPECT_EQ(delivered_, 1);
  EXPECT_EQ(follow_count_, 2);
}

TEST_F(NavigationTest, ExhaustedPickupRetriesDoNotPoisonNextJob) {
  reject_first_ = 2;
  start_mission(2);
  ASSERT_TRUE(until([this] { return mission_status_ == "fault"; }, 4000ms));
  EXPECT_EQ(plan_count_, 4);
  EXPECT_EQ(delivered_, 1);
  EXPECT_EQ(follow_count_, 2);
}

TEST_F(NavigationTest, LoadedFailureDoesNotSelectAnotherJob) {
  // Обе попытки dropoff отклоняются планировщиком после успешного pickup.
  start_mission(2);
  ASSERT_TRUE(until([this] { return follow_count_ == 1; }));
  reject_plan_ = true;
  ASSERT_TRUE(until([this] { return mission_status_ == "fault"; }, 3500ms));
  EXPECT_EQ(delivered_, 0);
  EXPECT_EQ(follow_count_, 1);
  EXPECT_EQ(plan_count_, 3);
}

TEST_F(NavigationTest, FrozenSimulationTimeCancelsActiveDelivery) {
  hold_follow_ = true;
  start_mission(1, true);
  ASSERT_TRUE(until([this] { return follow_count_ == 1; }));
  freeze_ = true;
  ASSERT_TRUE(until([this] { return mission_status_ == "cancelled"; }, 1500ms));
  EXPECT_EQ(delivered_, 0);
  EXPECT_EQ(cancel_count_, 1);
}

TEST_F(NavigationTest, ManualResetPreparesIdleEpisodeThenStartKeepsItsId) {
  start_mission(1, false, false);
  auto reset = server_->create_client<std_srvs::srv::Trigger>("mission/reset");
  ASSERT_TRUE(until([&reset] { return reset->service_is_ready(); }));
  auto reset_result = reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(until([&reset_result] { return reset_result.wait_for(0ms) == std::future_status::ready; }));
  ASSERT_TRUE(reset_result.get()->success);
  ASSERT_TRUE(until([this] { return mission_status_ == "idle" && !episode_id_.empty(); }));
  const auto prepared_id = episode_id_;
  EXPECT_EQ(plan_count_, 0);
  auto start = server_->create_client<std_srvs::srv::Trigger>("mission/start");
  ASSERT_TRUE(until([&start] { return start->service_is_ready(); }));
  auto start_result = start->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(until([&start_result] { return start_result.wait_for(0ms) == std::future_status::ready; }));
  ASSERT_TRUE(start_result.get()->success);
  ASSERT_TRUE(until([this] { return mission_status_ == "completed"; }, 2500ms));
  EXPECT_EQ(episode_id_, prepared_id);
}

TEST_F(NavigationTest, LiveParameterChangesAreRejected) {
  start_mission();
  const auto result = mission_->set_parameters_atomically({rclcpp::Parameter("method", "learned")});
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(mission_->method(), "classical");
}

TEST_F(NavigationTest, ClockResetCancelsOldGoalBeforeNewEpisode) {
  hold_follow_ = true;
  start_mission(1, true);
  ASSERT_TRUE(until([this] { return follow_count_ == 1; }));
  const auto old_episode = episode_id_;
  sim_time_ = 0.1;
  ASSERT_TRUE(until([this, &old_episode] { return episode_id_ != old_episode; }, 1500ms));
  EXPECT_EQ(cancel_count_, 1);
  hold_follow_ = false;
  ASSERT_TRUE(until([this] { return mission_status_ == "completed"; }, 2500ms));
  EXPECT_EQ(delivered_, 1);
}
}  // namespace
