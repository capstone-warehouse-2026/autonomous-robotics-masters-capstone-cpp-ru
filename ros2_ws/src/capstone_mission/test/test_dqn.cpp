#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <yaml-cpp/yaml.h>
#include "capstone_mission/dispatch.hpp"

namespace cm = capstone_mission;
namespace {
const std::string models = std::string(CAPSTONE_MISSION_SOURCE_DIR) + "/models/";
TEST(Dqn, MatchesPythonExportOnHeldOutStates) {
  cm::DqnModel model(models + "dispatch_dqn.json");
  const auto golden = YAML::LoadFile(models + "golden.json");
  ASSERT_GE(golden["cases"].size(), 20U);
  for (const auto &entry : golden["cases"]) {
    const auto values = entry["observation"].as<std::vector<float>>();
    ASSERT_EQ(values.size(), 32U);
    cm::Observation observation;
    std::copy(values.begin(), values.end(), observation.begin());
    const auto actual = model.predict(observation);
    const auto expected = entry["q_values"].as<std::vector<float>>();
    const auto mask = entry["mask"].as<std::vector<bool>>();
    ASSERT_EQ(expected.size(), 5U);
    int action = 4;
    float maximum = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < 5; ++i) {
      EXPECT_NEAR(actual[i], expected[i], golden["tolerance"].as<float>());
      if (mask[i] && actual[i] > maximum) { maximum = actual[i]; action = i; }
    }
    EXPECT_EQ(action, entry["action"].as<int>());
  }
}
TEST(Dqn, EveryTrainingSeedLoadsAndRejectsNan) {
  for (const auto &prefix : {"dqn_seed_", "dqn_no_congestion_seed_"}) {
    for (int seed = 0; seed < 3; ++seed) {
      cm::DqnModel model(models + prefix + std::to_string(seed) + ".json");
      cm::Observation observation{};
      for (float value : model.predict(observation)) EXPECT_TRUE(std::isfinite(value));
      observation[0] = std::numeric_limits<float>::quiet_NaN();
      EXPECT_THROW(model.predict(observation), std::invalid_argument);
    }
  }
}
TEST(Dqn, InvalidShapeAndNonfiniteWeightsAreRejected) {
  auto root = YAML::LoadFile(models + "dispatch_dqn.json");
  const auto path = std::filesystem::temp_directory_path() / "capstone_g5_invalid_model.yaml";
  root["input_size"] = 31;
  { std::ofstream file(path); file << root; }
  EXPECT_THROW(cm::DqnModel{path.string()}, std::invalid_argument);
  root["input_size"] = 32;
  root["b2"][0] = std::numeric_limits<float>::infinity();
  { std::ofstream file(path); file << root; }
  EXPECT_THROW(cm::DqnModel{path.string()}, std::invalid_argument);
  std::filesystem::remove(path);
}
TEST(Dqn, MasksUnoccupiedSlotsEvenWhenTheyHaveLargestQValue) {
  auto root = YAML::LoadFile(models + "dispatch_dqn.json");
  for (const auto &key : {"w1", "b1", "w2", "b2"}) {
    for (std::size_t i = 0; i < root[key].size(); ++i) root[key][i] = 0.0;
  }
  root["b2"][0] = 1.0;
  root["b2"][3] = 100.0;
  const auto path = std::filesystem::temp_directory_path() / "capstone_g5_mask_model.yaml";
  { std::ofstream file(path); file << root; }
  cm::DqnModel model(path.string());
  cm::Job job;
  job.id = "a"; job.state = cm::JobState::Released;
  auto decision = cm::choose_action({&job}, {{1, 1, 1, 1}}, {}, &model, true);
  EXPECT_EQ(decision.action, 0);
  EXPECT_FALSE(decision.fallback);
  std::filesystem::remove(path);
}
}  // namespace

TEST(Preprocessing, MatchesPythonFromRawScenarios) {
  const auto fixture = YAML::LoadFile(models + "preprocessing_golden.json");
  const auto path = std::filesystem::temp_directory_path() / "capstone_g5_preprocess.yaml";
  for (const auto &entry : fixture["cases"]) {
    { std::ofstream file(path); file << entry["scenario"]; }
    auto scenario = cm::load_scenario(path.string());
    const auto raw = entry["context"];
    cm::DispatchContext context{raw["now"].as<double>(), raw["robot_x"].as<double>(),
      raw["robot_y"].as<double>(), raw["loaded"].as<bool>(), raw["navigation_failures"].as<int>()};
    cm::JobQueue queue(scenario.jobs);
    queue.release(context.now);
    const auto candidates = queue.candidates();
    const auto expected_ids = entry["candidate_ids"].as<std::vector<std::string>>();
    ASSERT_EQ(candidates.size(), expected_ids.size());
    for (std::size_t i = 0; i < candidates.size(); ++i)
      EXPECT_EQ(candidates[i]->id, expected_ids[i]);
    const auto travel = cm::estimate_travel(scenario, candidates, context,
                                           entry["current_station"].as<std::string>());
    const auto observation = cm::make_observation(candidates, travel, context,
                                                  entry["congestion_feature"].as<bool>());
    const auto expected = entry["observation"].as<std::vector<float>>();
    for (std::size_t i = 0; i < observation.size(); ++i)
      EXPECT_NEAR(observation[i], expected.at(i), fixture["tolerance"].as<double>());
    const auto mask = cm::make_action_mask(candidates, context.loaded);
    const auto expected_mask = entry["mask"].as<std::vector<bool>>();
    for (std::size_t i = 0; i < mask.size(); ++i) EXPECT_EQ(mask[i], expected_mask.at(i));
    EXPECT_EQ(cm::classical_action(candidates, travel), entry["classical_action"].as<int>());
  }
  std::filesystem::remove(path);
}
