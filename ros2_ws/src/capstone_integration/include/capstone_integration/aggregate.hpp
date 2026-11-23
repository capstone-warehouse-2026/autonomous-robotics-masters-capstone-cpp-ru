#pragma once

#include <string>
#include <utility>
#include <vector>

#include "capstone_integration/episode_record.hpp"

namespace capstone_integration {

// Aggregated metrics of one (config_id, scenario_id) group, docs/EXPERIMENTS.md.
struct SummaryRow {
  std::string config_id;
  std::string scenario_id;
  int episodes{0};
  int success{0};
  int timeout{0};
  int aborted{0};
  int collision{0};
  double success_rate{0.0};
  double ci_low{0.0};
  double ci_high{0.0};
  double mean_elapsed_s{0.0};
  double p95_elapsed_s{0.0};
  double mean_distance_m{0.0};
};

// Wilson score interval for a binomial fraction (docs/EXPERIMENTS.md asks for it).
std::pair<double, double> wilson_interval(int successes, int total, double z = 1.96);

// Nearest-rank percentile; `fraction` in [0, 1]. Empty input returns 0.
double percentile(std::vector<double> values, double fraction);

std::vector<SummaryRow> aggregate(const std::vector<EpisodeRecord> &records);

std::string summary_csv_header();
std::string summary_csv_row(const SummaryRow &row);

}  // namespace capstone_integration
