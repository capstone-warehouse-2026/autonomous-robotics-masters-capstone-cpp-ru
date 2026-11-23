#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_integration/episode_record.hpp"

namespace {

using capstone_integration::csv_header;
using capstone_integration::EpisodeRecord;
using capstone_integration::to_csv_row;

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> fields;
  std::string current;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  fields.push_back(current);
  return fields;
}

TEST(EpisodeCsv, HeaderMatchesTheDocumentedSchema) {
  const auto fields = split(csv_header());
  ASSERT_EQ(fields.size(), 18U);
  EXPECT_EQ(fields.front(), "episode_id");
  EXPECT_EQ(fields.back(), "placeholder_components");
}

TEST(EpisodeCsv, RowHasTheSameNumberOfFields) {
  const EpisodeRecord record;
  EXPECT_EQ(split(to_csv_row(record)).size(), split(csv_header()).size());
}

TEST(EpisodeCsv, PlaceholderListIsPartOfTheRow) {
  const EpisodeRecord record;
  const auto fields = split(to_csv_row(record));
  EXPECT_EQ(fields.back(), record.placeholder_components);
  EXPECT_NE(fields.back().find("g1_sensors"), std::string::npos);
  EXPECT_NE(fields.back().find("g5_mission"), std::string::npos);
}

TEST(EpisodeCsv, CommasInValuesDoNotBreakTheSchema) {
  EpisodeRecord record;
  record.episode_id = "a,b";
  record.terminal_reason = "SUCCESS, but noisy";
  const auto fields = split(to_csv_row(record));
  EXPECT_EQ(fields.size(), 18U);
  EXPECT_EQ(fields.front(), "a;b");
  EXPECT_EQ(fields[6], "SUCCESS; but noisy");
}

TEST(EpisodeCsv, FormatsNumbers) {
  EpisodeRecord record;
  record.elapsed_sim_s = 12.3456;
  record.distance_m = 3.1;
  const std::string row = to_csv_row(record);
  EXPECT_NE(row.find(",12.346,"), std::string::npos);
  EXPECT_NE(row.find(",3.100,"), std::string::npos);
}

TEST(EpisodeCsv, RejectsInvalidRecords) {
  EpisodeRecord record;
  record.episode_id.clear();
  EXPECT_THROW(to_csv_row(record), std::invalid_argument);

  EpisodeRecord negative;
  negative.elapsed_sim_s = -0.1;
  EXPECT_THROW(to_csv_row(negative), std::invalid_argument);
}

}  // namespace
