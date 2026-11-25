#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_integration/aggregate.hpp"
#include "capstone_integration/episode_record.hpp"

namespace {

using capstone_integration::aggregate;
using capstone_integration::EpisodeRecord;
using capstone_integration::parse_csv_row;
using capstone_integration::percentile;
using capstone_integration::summary_csv_header;
using capstone_integration::summary_csv_row;
using capstone_integration::to_csv_row;
using capstone_integration::wilson_interval;

TEST(ParseCsvRow, RoundTripsTheRecord) {
  EpisodeRecord record;
  record.episode_id = "e-1";
  record.config_id = "C0";
  record.scenario_id = "S1";
  record.terminal_reason = "SUCCESS";
  record.elapsed_sim_s = 17.25;
  record.delivered = 1;
  record.distance_m = 7.543;
  record.placeholder_components = "g1_sensors;g5_mission";

  const auto parsed = parse_csv_row(to_csv_row(record));
  EXPECT_EQ(parsed.episode_id, record.episode_id);
  EXPECT_EQ(parsed.config_id, record.config_id);
  EXPECT_EQ(parsed.scenario_id, record.scenario_id);
  EXPECT_EQ(parsed.terminal_reason, record.terminal_reason);
  EXPECT_NEAR(parsed.elapsed_sim_s, record.elapsed_sim_s, 0.001);
  EXPECT_EQ(parsed.delivered, record.delivered);
  EXPECT_NEAR(parsed.distance_m, record.distance_m, 0.001);
  EXPECT_EQ(parsed.placeholder_components, record.placeholder_components);
}

TEST(ParseCsvRow, RejectsBrokenInput) {
  EXPECT_THROW(parse_csv_row("a,b,c"), std::invalid_argument);
  std::string row = to_csv_row(EpisodeRecord{});
  const auto position = row.find(",ABORTED,");
  ASSERT_NE(position, std::string::npos);
  row.replace(position, 9, ",ABORTED,x");
  EXPECT_THROW(parse_csv_row(row), std::invalid_argument);
}

TEST(WilsonInterval, KnownValues) {
  const auto all = wilson_interval(10, 10);
  EXPECT_NEAR(all.first, 0.7225, 0.002);
  EXPECT_NEAR(all.second, 1.0, 1e-9);

  const auto none = wilson_interval(0, 10);
  EXPECT_NEAR(none.first, 0.0, 1e-9);
  EXPECT_NEAR(none.second, 0.2775, 0.002);

  const auto half = wilson_interval(5, 10);
  EXPECT_NEAR(half.first, 0.2366, 0.002);
  EXPECT_NEAR(half.second, 0.7634, 0.002);

  EXPECT_EQ(wilson_interval(0, 0), std::make_pair(0.0, 0.0));
  EXPECT_THROW(wilson_interval(11, 10), std::invalid_argument);
}

TEST(Percentile, NearestRank) {
  EXPECT_DOUBLE_EQ(percentile({10.0, 20.0, 30.0, 40.0}, 0.5), 20.0);
  EXPECT_DOUBLE_EQ(percentile({10.0, 20.0, 30.0, 40.0}, 0.95), 40.0);
  EXPECT_DOUBLE_EQ(percentile({10.0, 20.0, 30.0, 40.0}, 0.0), 10.0);
  EXPECT_DOUBLE_EQ(percentile({}, 0.5), 0.0);
  EXPECT_THROW(percentile({1.0}, 1.5), std::invalid_argument);
}

TEST(Aggregate, GroupsAndCounts) {
  EpisodeRecord success_a;
  success_a.episode_id = "a";
  success_a.config_id = "C0";
  success_a.terminal_reason = "SUCCESS";
  success_a.elapsed_sim_s = 10.0;
  success_a.distance_m = 1.0;

  EpisodeRecord success_b = success_a;
  success_b.episode_id = "b";
  success_b.elapsed_sim_s = 20.0;
  success_b.distance_m = 2.0;

  EpisodeRecord timeout = success_a;
  timeout.episode_id = "c";
  timeout.terminal_reason = "TIMEOUT";
  timeout.elapsed_sim_s = 30.0;
  timeout.distance_m = 3.0;

  EpisodeRecord other = success_a;
  other.episode_id = "d";
  other.config_id = "C6";
  other.elapsed_sim_s = 40.0;

  const auto rows = aggregate({success_a, success_b, timeout, other});
  ASSERT_EQ(rows.size(), 2U);
  const auto &c0 = rows.front();
  EXPECT_EQ(c0.config_id, "C0");
  EXPECT_EQ(c0.episodes, 3);
  EXPECT_EQ(c0.success, 2);
  EXPECT_EQ(c0.timeout, 1);
  EXPECT_NEAR(c0.success_rate, 2.0 / 3.0, 1e-9);
  EXPECT_NEAR(c0.mean_elapsed_s, 20.0, 1e-9);
  EXPECT_NEAR(c0.p95_elapsed_s, 30.0, 1e-9);
  EXPECT_NEAR(c0.mean_distance_m, 2.0, 1e-9);
  EXPECT_GT(c0.ci_low, 0.0);
  EXPECT_LT(c0.ci_high, 1.0);
}

TEST(SummaryCsv, HasStableSchema) {
  capstone_integration::SummaryRow row;
  row.config_id = "C0";
  row.scenario_id = "S1";
  row.episodes = 3;
  row.success = 2;
  const std::string line = summary_csv_row(row);
  std::size_t fields = 1;
  for (char c : line) {
    fields += c == ',' ? 1 : 0;
  }
  EXPECT_EQ(fields, 13U);
  std::size_t header_fields = 1;
  for (char c : summary_csv_header()) {
    header_fields += c == ',' ? 1 : 0;
  }
  EXPECT_EQ(header_fields, 13U);
}

}  // namespace
