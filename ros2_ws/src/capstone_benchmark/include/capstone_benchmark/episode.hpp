#pragma once
#include <functional>
#include <string>
#include <vector>
#include "capstone_mission/dispatch.hpp"

namespace capstone_benchmark {
struct Event {
  std::string episode_id, job_id;
  double sim_time{0};
  std::string event, reason, target;
};
struct Metrics {
  std::string episode_id, family, method, terminal_reason;
  int released{0}, delivered{0}, on_time{0}, failed{0}, unfinished{0};
  int recoveries{0}, fallbacks{0}, decisions{0};
  double elapsed{0}, distance{0}, idle{0}, tardiness_mean{0}, tardiness_p95{0};
  double unfinished_age_mean{0}, decision_latency_p95_ms{0};
  double throughput() const;
  double on_time_fraction() const;
};
struct EpisodeResult { Metrics metrics; std::vector<Event> events; };
EpisodeResult run_episode(const capstone_mission::Scenario &scenario, const std::string &method,
                          const capstone_mission::DqnModel *model = nullptr,
                          bool congestion_feature = true);
std::string csv_escape(const std::string &value);
std::vector<std::string> parse_csv_row(const std::string &line);
std::string metrics_header();
std::string metrics_row(const Metrics &metrics);
std::pair<double, double> bootstrap_mean_ci(const std::vector<double> &values,
                                          unsigned seed = 2026, int samples = 10000);
}  // namespace capstone_benchmark
