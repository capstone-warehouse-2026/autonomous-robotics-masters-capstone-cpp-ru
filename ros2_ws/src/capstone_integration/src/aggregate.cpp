#include "capstone_integration/aggregate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <numeric>
#include <stdexcept>

namespace capstone_integration {

namespace {

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

}  // namespace

std::pair<double, double> wilson_interval(int successes, int total, double z) {
  if (total <= 0) {
    return {0.0, 0.0};
  }
  if (successes < 0 || successes > total) {
    throw std::invalid_argument("wilson_interval: successes outside [0, total]");
  }
  if (!(z > 0.0)) {
    throw std::invalid_argument("wilson_interval: z must be positive");
  }
  const double n = static_cast<double>(total);
  const double p = static_cast<double>(successes) / n;
  const double z2 = z * z;
  const double denominator = 1.0 + z2 / n;
  const double center = (p + z2 / (2.0 * n)) / denominator;
  const double margin = z * std::sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n)) / denominator;
  return {std::max(0.0, center - margin), std::min(1.0, center + margin)};
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  if (fraction < 0.0 || fraction > 1.0) {
    throw std::invalid_argument("percentile: fraction must be in [0, 1]");
  }
  std::sort(values.begin(), values.end());
  const auto n = static_cast<std::size_t>(values.size());
  const auto rank = static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(n)));
  const std::size_t index = rank == 0 ? 0 : std::min(rank - 1, n - 1);
  return values[index];
}

std::vector<SummaryRow> aggregate(const std::vector<EpisodeRecord> &records) {
  std::map<std::pair<std::string, std::string>, std::vector<const EpisodeRecord *>> groups;
  for (const auto &record : records) {
    groups[{record.config_id, record.scenario_id}].push_back(&record);
  }

  std::vector<SummaryRow> rows;
  rows.reserve(groups.size());
  for (const auto &entry : groups) {
    SummaryRow row;
    row.config_id = entry.first.first;
    row.scenario_id = entry.first.second;
    row.episodes = static_cast<int>(entry.second.size());
    std::vector<double> elapsed;
    std::vector<double> distance;
    for (const auto *record : entry.second) {
      row.success += record->terminal_reason == "SUCCESS" ? 1 : 0;
      row.timeout += record->terminal_reason == "TIMEOUT" ? 1 : 0;
      row.aborted += record->terminal_reason == "ABORTED" ? 1 : 0;
      row.collision += record->terminal_reason == "COLLISION" ? 1 : 0;
      elapsed.push_back(record->elapsed_sim_s);
      distance.push_back(record->distance_m);
    }
    row.success_rate = static_cast<double>(row.success) / static_cast<double>(row.episodes);
    const auto interval = wilson_interval(row.success, row.episodes);
    row.ci_low = interval.first;
    row.ci_high = interval.second;
    row.mean_elapsed_s =
        std::accumulate(elapsed.begin(), elapsed.end(), 0.0) / static_cast<double>(elapsed.size());
    row.p95_elapsed_s = percentile(elapsed, 0.95);
    row.mean_distance_m =
        std::accumulate(distance.begin(), distance.end(), 0.0) /
        static_cast<double>(distance.size());
    rows.push_back(row);
  }
  return rows;
}

std::string summary_csv_header() {
  return "config_id,scenario_id,episodes,success,success_rate,ci_low,ci_high,timeout,aborted,"
         "collision,mean_elapsed_s,p95_elapsed_s,mean_distance_m";
}

std::string summary_csv_row(const SummaryRow &row) {
  std::string out;
  out += row.config_id + ',';
  out += row.scenario_id + ',';
  out += std::to_string(row.episodes) + ',';
  out += std::to_string(row.success) + ',';
  out += number(row.success_rate) + ',';
  out += number(row.ci_low) + ',';
  out += number(row.ci_high) + ',';
  out += std::to_string(row.timeout) + ',';
  out += std::to_string(row.aborted) + ',';
  out += std::to_string(row.collision) + ',';
  out += number(row.mean_elapsed_s) + ',';
  out += number(row.p95_elapsed_s) + ',';
  out += number(row.mean_distance_m);
  return out;
}

}  // namespace capstone_integration
