#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include "capstone_mission/domain.hpp"
#include "capstone_mission/dispatch.hpp"

namespace cm = capstone_mission;
namespace {
cm::Job job(std::string id, double release, double due) {
  cm::Job value;
  value.id = std::move(id);
  value.pickup = "a";
  value.dropoff = "b";
  value.release_time = release;
  value.due_time = due;
  return value;
}
TEST(Queue, ReleasesOnlyPastAndRetainsOverflow) {
  cm::JobQueue queue({job("f", 10, 20), job("e", 0, 5), job("d", 0, 5),
                      job("c", 0, 5), job("b", 0, 5), job("a", 0, 5)});
  queue.release(0);
  auto candidates = queue.candidates();
  ASSERT_EQ(candidates.size(), 4U);
  EXPECT_EQ(candidates.front()->id, "a");
  EXPECT_EQ(candidates.back()->id, "d");
  EXPECT_EQ(queue.find("e").state, cm::JobState::Released);
  EXPECT_EQ(queue.find("f").state, cm::JobState::Unreleased);
  EXPECT_EQ(queue.jobs().size(), 6U);
}
TEST(Queue, DeliveryRequiresPickupAndPreventsConcurrentLoad) {
  cm::JobQueue queue({job("a", 0, 100), job("b", 0, 100)});
  queue.release(0);
  EXPECT_THROW(queue.mark_delivered("a", 1), std::invalid_argument);
  queue.mark_active("a");
  EXPECT_THROW(queue.mark_active("b"), std::invalid_argument);
  queue.mark_picked("a", 2);
  EXPECT_THROW(queue.mark_active("b"), std::invalid_argument);
  EXPECT_THROW(queue.mark_delivered("a", 1), std::invalid_argument);
  queue.mark_delivered("a", 3);
  queue.mark_active("b");
  EXPECT_EQ(queue.find("a").delivered_time, 3);
}
TEST(Queue, ResetDiscardsEpisodeState) {
  cm::JobQueue queue({job("a", 0, 1)});
  queue.release(0);
  queue.mark_active("a");
  queue.mark_failed("a", "no_path");
  EXPECT_TRUE(queue.all_terminal());
  queue.reset();
  EXPECT_FALSE(queue.all_terminal());
  EXPECT_EQ(queue.find("a").state, cm::JobState::Unreleased);
  EXPECT_TRUE(queue.find("a").failure_reason.empty());
}
TEST(Queue, FutureJobsKeepEpisodeOpen) {
  cm::JobQueue queue({job("a", 10, 20)});
  queue.release(0);
  EXPECT_TRUE(queue.candidates().empty());
  EXPECT_FALSE(queue.all_terminal());
}
TEST(Queue, RejectsInvalidInput) {
  EXPECT_THROW(cm::JobQueue({job("a", 0, 1), job("a", 0, 2)}), std::invalid_argument);
  EXPECT_THROW(cm::JobQueue({job("a", -1, 1)}), std::invalid_argument);
  cm::JobQueue queue;
  EXPECT_THROW(queue.release(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(queue.find("unknown"), std::invalid_argument);
}
TEST(Classical, DeadlineThenRouteThenId) {
  cm::JobQueue queue({job("a", 0, 120), job("b", 0, 90), job("c", 0, 90)});
  queue.release(0);
  auto candidates = queue.candidates();
  std::vector<cm::TravelEstimate> travel{{0, 0, 15, 1}, {0, 0, 25, 1}, {0, 0, 10, 1}};
  EXPECT_EQ(cm::classical_action(candidates, travel), 2);
  travel[1].route_length = 10;
  EXPECT_EQ(cm::classical_action(candidates, travel), 1);
}
TEST(Classical, EmptyQueueWaits) { EXPECT_EQ(cm::classical_action({}, {}), 4); }
TEST(Observation, SharesExactUnitsAndMasks) {
  cm::JobQueue queue({job("a", 5, 100)});
  queue.release(10);
  const auto candidates = queue.candidates();
  cm::DispatchContext context{10, 2, -4, false, 2};
  auto values = cm::make_observation(candidates, {{12, 24, 18, 1.5}}, context);
  EXPECT_FLOAT_EQ(values[0], 0.1F);
  EXPECT_FLOAT_EQ(values[1], 0.2F);
  EXPECT_FLOAT_EQ(values[2], 52.0F / 300.0F);
  EXPECT_FLOAT_EQ(values[3], 5.0F / 300.0F);
  EXPECT_FLOAT_EQ(values[4], 0.5F);
  EXPECT_FLOAT_EQ(values[5], 1.0F);
  EXPECT_FLOAT_EQ(values[6], 1.0F);
  EXPECT_FLOAT_EQ(values[7], 0.0F);
  EXPECT_FLOAT_EQ(values[28], 0.1F);
  EXPECT_FLOAT_EQ(values[29], -0.2F);
  const auto mask = cm::make_action_mask(candidates);
  EXPECT_TRUE(mask[0]); EXPECT_TRUE(mask[4]); EXPECT_FALSE(mask[1]);
  EXPECT_FALSE(cm::make_action_mask(candidates, true)[0]);
  values = cm::make_observation(candidates, {{12, 24, 18, 1.5}}, context, false);
  EXPECT_FLOAT_EQ(values[4], 0.0F);
}
TEST(Observation, FutureInformationRejected) {
  cm::JobQueue queue({job("a", 10, 100)});
  queue.release(10);
  EXPECT_THROW(cm::make_observation(queue.candidates(), {{1, 1, 1, 1}}, {0}),
               std::invalid_argument);
}
TEST(Dispatch, MissingModelFallsBackButClassicalDoesNotNeedModel) {
  cm::JobQueue queue({job("a", 0, 100)});
  queue.release(0);
  auto result = cm::choose_action(queue.candidates(), {{1, 1, 1, 1}}, {}, nullptr, true);
  EXPECT_EQ(result.action, 0); EXPECT_TRUE(result.fallback); EXPECT_FALSE(result.reason.empty());
  result = cm::choose_action(queue.candidates(), {{1, 1, 1, 1}}, {}, nullptr, false);
  EXPECT_EQ(result.action, 0); EXPECT_FALSE(result.fallback);
}
TEST(Dispatch, LoadedRobotCannotSelectAnotherJob) {
  cm::JobQueue queue({job("a", 0, 100)});
  queue.release(0);
  cm::DispatchContext context;
  context.loaded = true;
  EXPECT_EQ(cm::choose_action(queue.candidates(), {{1, 1, 1, 1}}, context, nullptr, false).action, 4);
}
TEST(Scenario, EstimatesUseTablesWithoutFutureEvents) {
  cm::Scenario scenario;
  scenario.stations = {{"a", 0, 0, 0}, {"b", 1, 0, 0}};
  scenario.initial_station = "a";
  scenario.jobs = {job("a", 0, 100)};
  scenario.travel_times = {{0, 10}, {10, 0}};
  scenario.congestion = {{1, 2}, {2, 1}};
  cm::validate_scenario(scenario);
  cm::JobQueue queue(scenario.jobs);
  queue.release(0);
  auto estimate = cm::estimate_travel(scenario, queue.candidates(), {}, "a");
  ASSERT_EQ(estimate.size(), 1U);
  EXPECT_EQ(estimate[0].delivery_time, 20);
  EXPECT_EQ(estimate[0].route_length, 5);
  scenario.travel_times[0].pop_back();
  EXPECT_THROW(cm::validate_scenario(scenario), std::invalid_argument);
}
TEST(Model, MissingModelIsReported) {
  EXPECT_THROW(cm::DqnModel("/tmp/g5-nonexistent-model.json"), std::exception);
}
}  // namespace
