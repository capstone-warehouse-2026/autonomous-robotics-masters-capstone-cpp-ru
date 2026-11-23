#include "capstone_integration/episode_record.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>

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

}  // namespace capstone_integration
