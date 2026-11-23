#include <gtest/gtest.h>
#include "capstone_benchmark/episode.hpp"
using namespace capstone_mission;
using namespace capstone_benchmark;
namespace {
Scenario scenario() {
  Scenario s;
  s.episode_id = "test";
  s.initial_station = "a";
  s.horizon = 20;
  s.stations = {{"a", 0, 0, 0}, {"b", 2, 0, 0}};
  s.travel_times = {{0, 4}, {4, 0}};
  s.congestion = {{1, 1}, {1, 1}};
  s.jobs = {{"one", "a", "b", 0, 6, 1, 1}};
  return s;
}
TEST(Episode, DeliveryDeadlineAndDistance) {
  auto result = run_episode(scenario(), "classical");
  EXPECT_EQ(result.metrics.delivered, 1);
  EXPECT_EQ(result.metrics.on_time, 1);
  EXPECT_DOUBLE_EQ(result.metrics.elapsed, 6);
  EXPECT_DOUBLE_EQ(result.metrics.distance, 2);
  EXPECT_EQ(result.metrics.terminal_reason, "completed");
  ASSERT_EQ(result.events.size(), 4u);
  EXPECT_EQ(result.events[2].event, "picked");
  EXPECT_EQ(result.events[3].event, "delivered");
}
TEST(Episode, UnfinishedJobsRemainInDenominatorAndLedger) {
  auto s = scenario();
  s.horizon = 3;
  s.jobs.push_back({"two", "b", "a", 2, 4, 1, 1});
  const auto result = run_episode(s, "classical");
  EXPECT_EQ(result.metrics.released, 2);
  EXPECT_EQ(result.metrics.delivered, 0);
  EXPECT_EQ(result.metrics.unfinished, 2);
  EXPECT_DOUBLE_EQ(result.metrics.distance, 1);
  EXPECT_DOUBLE_EQ(result.metrics.unfinished_age_mean, 2);
  EXPECT_NE(metrics_row(result.metrics).find("NA"), std::string::npos);
}
TEST(Episode, FutureJobsDoNotDisappearOnEarlyCompletion) {
  auto s = scenario();
  s.jobs.push_back({"future", "b", "a", 10, 18, 1, 1});
  auto result = run_episode(s, "classical");
  EXPECT_EQ(result.metrics.delivered, 2);
  EXPECT_EQ(result.metrics.released, 2);
  EXPECT_DOUBLE_EQ(result.metrics.idle, 4);
}
TEST(Episode, MissingModelHasExplicitFallback) {
  auto result = run_episode(scenario(), "learned", nullptr);
  EXPECT_EQ(result.metrics.fallbacks, 1);
  EXPECT_EQ(result.metrics.delivered, 1);
}
TEST(Episode, BlockedDropoffRetainsFutureWorkAndCountsRecovery) {
  auto s = scenario();
  s.blocked = {{false, true}, {false, false}};
  s.jobs.push_back({"future", "b", "a", 10, 18, 1, 1});
  const auto result = run_episode(s, "classical");
  EXPECT_EQ(result.metrics.failed, 1);
  EXPECT_EQ(result.metrics.recoveries, 1);
  EXPECT_EQ(result.metrics.released, 2);
  EXPECT_EQ(result.metrics.delivered, 0);
  EXPECT_EQ(result.metrics.unfinished, 2);
  EXPECT_DOUBLE_EQ(result.metrics.elapsed, 20);
  EXPECT_DOUBLE_EQ(result.metrics.distance, 0);
  EXPECT_EQ(result.metrics.terminal_reason, "navigation_failure");
}
TEST(Statistics, ConstantBootstrapAndQuotedCsv) {
  const auto ci = bootstrap_mean_ci({0.5, 0.5, 0.5});
  EXPECT_DOUBLE_EQ(ci.first, 0.5);
  EXPECT_DOUBLE_EQ(ci.second, 0.5);
  EXPECT_EQ(parse_csv_row(csv_escape("a,\"b") + ",c")[0], "a,\"b");
  EXPECT_THROW(parse_csv_row("\"broken"), std::invalid_argument);
}
}
