#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_integration/audit_core.hpp"

namespace {

using capstone_integration::Contract;
using capstone_integration::evaluate_action;
using capstone_integration::evaluate_clock;
using capstone_integration::evaluate_ground_truth;
using capstone_integration::evaluate_placeholders;
using capstone_integration::evaluate_tf;
using capstone_integration::evaluate_topic;
using capstone_integration::exit_code;
using capstone_integration::format_table;
using capstone_integration::Row;
using capstone_integration::to_json;
using capstone_integration::TopicSample;
using capstone_integration::Verdict;

Contract make_contract() {
  Contract contract;
  contract.name = "/sensors/scan";
  contract.type = "sensor_msgs/msg/LaserScan";
  contract.min_hz = 10.0;
  contract.max_hz = 10.0;
  contract.frame = "base_link";
  contract.min_publishers = 1;
  contract.max_publishers = 1;
  return contract;
}

TopicSample make_sample() {
  TopicSample sample;
  sample.contract = make_contract();
  sample.exists = true;
  sample.actual_type = "sensor_msgs/msg/LaserScan";
  sample.publishers = 1;
  sample.messages = 50;
  sample.measured_hz = 10.0;
  sample.frame = "base_link";
  return sample;
}

TEST(EvaluateTopic, HealthyTopicIsOk) {
  EXPECT_EQ(evaluate_topic(make_sample()).verdict, Verdict::Ok);
}

TEST(EvaluateTopic, MissingTopicFails) {
  auto sample = make_sample();
  sample.exists = false;
  const auto row = evaluate_topic(sample);
  EXPECT_EQ(row.verdict, Verdict::Fail);
  EXPECT_EQ(row.detail, "topic not found");
}

TEST(EvaluateTopic, OptionalMissingTopicIsInfo) {
  auto sample = make_sample();
  sample.exists = false;
  sample.contract.required = false;
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Info);
}

TEST(EvaluateTopic, WrongTypeFails) {
  auto sample = make_sample();
  sample.actual_type = "nav_msgs/msg/Odometry";
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Fail);
}

TEST(EvaluateTopic, NoPublisherFails) {
  auto sample = make_sample();
  sample.publishers = 0;
  const auto row = evaluate_topic(sample);
  EXPECT_EQ(row.verdict, Verdict::Fail);
  EXPECT_NE(row.detail.find("0 publisher"), std::string::npos);
}

TEST(EvaluateTopic, DuplicatePublisherFails) {
  auto sample = make_sample();
  sample.publishers = 2;
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Fail);
}

TEST(EvaluateTopic, NoMessagesFails) {
  auto sample = make_sample();
  sample.messages = 0;
  sample.measured_hz = 0.0;
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Fail);
}

TEST(EvaluateTopic, WrongFrameFails) {
  auto sample = make_sample();
  sample.frame = "odom";
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Fail);
}

TEST(EvaluateTopic, WrongRateWarns) {
  auto sample = make_sample();
  sample.messages = 10;
  sample.measured_hz = 2.0;
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Warn);
}

TEST(EvaluateTopic, PresenceOnlyTopicDoesNotNeedMessages) {
  auto sample = make_sample();
  sample.contract.presence_only = true;
  sample.messages = 0;
  sample.measured_hz = 0.0;
  EXPECT_EQ(evaluate_topic(sample).verdict, Verdict::Ok);
}

TEST(EvaluateClock, Matrix) {
  EXPECT_EQ(evaluate_clock(500, 1.0, 6.0, true, 1).verdict, Verdict::Ok);
  EXPECT_EQ(evaluate_clock(0, 0.0, -1.0, true, 1).verdict, Verdict::Fail);
  EXPECT_EQ(evaluate_clock(500, 6.0, 6.0, true, 1).verdict, Verdict::Fail);
  EXPECT_EQ(evaluate_clock(500, 6.0, 5.0, false, 1).verdict, Verdict::Fail);
  EXPECT_EQ(evaluate_clock(500, 1.0, 6.0, true, 2).verdict, Verdict::Fail);
}

TEST(EvaluateActionsAndTf, PresentOrMissing) {
  EXPECT_EQ(evaluate_action("/planning/compute_path", true).verdict, Verdict::Ok);
  EXPECT_EQ(evaluate_action("/planning/compute_path", false).verdict, Verdict::Fail);
  EXPECT_EQ(evaluate_tf(true, true, true).verdict, Verdict::Ok);
  EXPECT_EQ(evaluate_tf(false, true, false).verdict, Verdict::Fail);
}

TEST(EvaluatePlaceholders, DependsOnTheRunMode) {
  EXPECT_EQ(evaluate_placeholders({"/perception", "/safety_monitor"}, false).verdict, Verdict::Ok);
  EXPECT_EQ(evaluate_placeholders({"/world_stub", "/perception"}, false).verdict, Verdict::Warn);
  EXPECT_EQ(evaluate_placeholders({"/world_stub"}, true).verdict, Verdict::Fail);
}

TEST(EvaluateGroundTruth, RuntimeSubscriberFails) {
  EXPECT_EQ(evaluate_ground_truth({}).verdict, Verdict::Ok);
  EXPECT_EQ(evaluate_ground_truth({"/benchmark_runner"}).verdict, Verdict::Ok);
  EXPECT_EQ(evaluate_ground_truth({"/world_stub"}).verdict, Verdict::Fail);
}

TEST(ReportFormat, TableAndJson) {
  const std::vector<Row> rows = {
      {"/sensors/scan", Verdict::Ok, "10.0 Hz"},
      {"/cmd_vel", Verdict::Fail, "0 publishers"},
  };
  const std::string table = format_table(rows);
  EXPECT_NE(table.find("OK"), std::string::npos);
  EXPECT_NE(table.find("FAIL"), std::string::npos);
  const std::string json = to_json(rows, 5.0, false);
  EXPECT_NE(json.find("\"failures\": 1"), std::string::npos);
  EXPECT_NE(json.find("\"measured_run\": false"), std::string::npos);
  EXPECT_NE(json.find("/cmd_vel"), std::string::npos);
}

TEST(ExitCode, FollowsFailOn) {
  const std::vector<Row> rows = {
      {"/cmd_vel", Verdict::Fail, ""},
      {"/sensors/scan", Verdict::Warn, ""},
      {"/map", Verdict::Ok, ""},
  };
  EXPECT_EQ(exit_code(rows, "never"), 0);
  EXPECT_EQ(exit_code(rows, "error"), 1);
  EXPECT_EQ(exit_code(rows, "warn"), 1);
  const std::vector<Row> healthy = {{"/map", Verdict::Ok, ""}};
  EXPECT_EQ(exit_code(healthy, "warn"), 0);
}

}  // namespace
