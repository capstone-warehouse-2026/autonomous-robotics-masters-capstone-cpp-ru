#pragma once

#include <string>

namespace capstone_integration {

// One row of the system benchmark CSV (docs/EXPERIMENTS.md, minimum schema).
// Fields that the skeleton cannot measure yet are written as "NA" instead of a fake zero.
struct EpisodeRecord {
  std::string episode_id{"stub-001"};
  std::string config_id{"SKELETON"};
  std::string scenario_id{"S1"};
  std::string layout_id{"warehouse"};
  int scenario_seed{0};
  std::string training_seed_bundle{"none"};
  std::string terminal_reason{"ABORTED"};  // SUCCESS | TIMEOUT | ABORTED | COLLISION
  double elapsed_sim_s{0.0};
  int delivered{0};
  int released{1};
  int on_time{0};
  int collision_count{0};
  double distance_m{0.0};
  std::string min_clearance_m{"NA"};
  int deadline_misses{0};
  int safety_interventions{0};
  int fallback_count{0};
  // Components that were simulated by the integration skeleton in this run. A measured run must
  // have an empty value here; the audit (next stage) refuses results with placeholders.
  std::string placeholder_components{"g1_sensors;g1_localization;g3_planning;g4_control;g5_mission"};
};

std::string csv_header();

// One CSV line, no trailing newline. Values never contain commas: the placeholder list uses ';'.
std::string to_csv_row(const EpisodeRecord &record);

// Parses one data line written by to_csv_row; throws std::invalid_argument on a schema mismatch.
EpisodeRecord parse_csv_row(const std::string &line);

}  // namespace capstone_integration
