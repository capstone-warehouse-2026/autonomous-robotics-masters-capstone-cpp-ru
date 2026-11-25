#include "capstone_integration/episode_record.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace capstone_integration {

namespace {

std::string sanitize(const std::string &value) {
  std::string out = value;
  for (char &c : out) {
    if (c == ',' || c == '\n' || c == '\r') {
      c = ';';
    }
  }
  return out;
}

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

}  // namespace

std::string csv_header() {
  return "episode_id,config_id,scenario_id,layout_id,scenario_seed,training_seed_bundle,"
         "terminal_reason,elapsed_sim_s,delivered,released,on_time,collision_count,distance_m,"
         "min_clearance_m,deadline_misses,safety_interventions,fallback_count,"
         "placeholder_components";
}

std::string to_csv_row(const EpisodeRecord &record) {
  if (record.episode_id.empty()) {
    throw std::invalid_argument("to_csv_row: episode_id must not be empty");
  }
  if (record.elapsed_sim_s < 0.0) {
    throw std::invalid_argument("to_csv_row: elapsed_sim_s must not be negative");
  }
  std::string row;
  row += sanitize(record.episode_id) + ',';
  row += sanitize(record.config_id) + ',';
  row += sanitize(record.scenario_id) + ',';
  row += sanitize(record.layout_id) + ',';
  row += std::to_string(record.scenario_seed) + ',';
  row += sanitize(record.training_seed_bundle) + ',';
  row += sanitize(record.terminal_reason) + ',';
  row += number(record.elapsed_sim_s) + ',';
  row += std::to_string(record.delivered) + ',';
  row += std::to_string(record.released) + ',';
  row += std::to_string(record.on_time) + ',';
  row += std::to_string(record.collision_count) + ',';
  row += number(record.distance_m) + ',';
  row += sanitize(record.min_clearance_m) + ',';
  row += std::to_string(record.deadline_misses) + ',';
  row += std::to_string(record.safety_interventions) + ',';
  row += std::to_string(record.fallback_count) + ',';
  row += sanitize(record.placeholder_components);
  return row;
}

EpisodeRecord parse_csv_row(const std::string &line) {
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

  constexpr std::size_t kFields = 18;
  if (fields.size() != kFields) {
    throw std::invalid_argument("parse_csv_row: expected " + std::to_string(kFields) +
                                " fields, got " + std::to_string(fields.size()));
  }
  EpisodeRecord record;
  try {
    record.episode_id = fields[0];
    record.config_id = fields[1];
    record.scenario_id = fields[2];
    record.layout_id = fields[3];
    record.scenario_seed = std::stoi(fields[4]);
    record.training_seed_bundle = fields[5];
    record.terminal_reason = fields[6];
    record.elapsed_sim_s = std::stod(fields[7]);
    record.delivered = std::stoi(fields[8]);
    record.released = std::stoi(fields[9]);
    record.on_time = std::stoi(fields[10]);
    record.collision_count = std::stoi(fields[11]);
    record.distance_m = std::stod(fields[12]);
    record.min_clearance_m = fields[13];
    record.deadline_misses = std::stoi(fields[14]);
    record.safety_interventions = std::stoi(fields[15]);
    record.fallback_count = std::stoi(fields[16]);
    record.placeholder_components = fields[17];
  } catch (const std::exception &) {
    throw std::invalid_argument("parse_csv_row: a numeric field is not a number");
  }
  if (record.episode_id.empty()) {
    throw std::invalid_argument("parse_csv_row: empty episode_id");
  }
  return record;
}

}  // namespace capstone_integration
