// Пересчёт closed-loop delivery metrics из неизменённых событий G5 и manifest.
#include "capstone_benchmark/episode.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
using namespace capstone_benchmark;
using namespace capstone_mission;
namespace {
double nonnegative_number(const std::string &value, const std::string &field) {
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result) || result < 0)
    throw std::invalid_argument("invalid " + field);
  return result;
}
bool terminal_event(const std::string &event) {
  return event == "episode_completed" || event == "episode_failed" ||
         event == "episode_cancelled" || event == "episode_fault";
}
}  // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 5) throw std::invalid_argument(
      "usage: mission_metrics scenario.yaml events.csv episode.yaml metrics.csv");
    const auto scenario = load_scenario(argv[1]);
    const auto run = YAML::LoadFile(argv[3]);
    std::ifstream input(argv[2]);
    if (!input) throw std::runtime_error("cannot read events");
    std::string line;
    std::getline(input, line);
    const auto header = parse_csv_row(line);
    if (header != std::vector<std::string>{"episode_id", "job_id", "sim_time", "event",
                                            "reason", "target"}) {
      throw std::invalid_argument("unexpected event schema");
    }
    std::vector<std::vector<std::string>> events;
    const std::string episode = run["episode_id"].as<std::string>();
    while (std::getline(input, line)) {
      auto fields = parse_csv_row(line);
      if (fields.size() != 6) throw std::invalid_argument("invalid event row");
      if (fields[0] != episode) continue;
      events.push_back(std::move(fields));
    }
    if (episode.empty() || events.empty()) throw std::invalid_argument("empty event log");
    Metrics metrics;
    metrics.episode_id = episode;
    metrics.family = scenario.family;
    metrics.method = run["method"].as<std::string>();
    metrics.terminal_reason = run["terminal_reason"].as<std::string>();
    metrics.elapsed = run["elapsed_sim_s"].as<double>();
    metrics.distance = run["distance_m"].as<double>();
    if (!std::isfinite(metrics.elapsed) || metrics.elapsed < 0 ||
        !std::isfinite(metrics.distance) || metrics.distance < 0)
      throw std::invalid_argument("invalid runner duration or distance");
    std::map<std::string, double> delivered;
    std::set<std::string> failed;
    std::vector<double> latencies;
    double idle_since = 0;
    double previous_time = 0;
    bool terminal_seen = false;
    std::string terminal_kind;
    bool complete_latencies = true;
    bool loaded_failure = false;
    std::string active_job;
    std::map<std::string, JobState> states;
    for (const auto &job : scenario.jobs) states.emplace(job.id, JobState::Unreleased);
    for (const auto &event : events) {
      const auto &id = event[1];
      const auto &kind = event[3];
      const double time = nonnegative_number(event[2], "event time");
      if (time < previous_time) throw std::invalid_argument("event time moved backwards");
      if (terminal_seen) throw std::invalid_argument("event after terminal event");
      previous_time = time;
      if (!id.empty() && !states.count(id)) throw std::invalid_argument("unknown job ID in ledger");
      const bool job_event = kind == "released" || kind == "selected" || kind == "picked" ||
                             kind == "delivered" || kind == "failed";
      if (job_event && id.empty()) throw std::invalid_argument("job event requires job ID");
      if (job_event) {
        const auto job = std::find_if(scenario.jobs.begin(), scenario.jobs.end(),
                                     [&](const Job &entry) { return entry.id == id; });
        if (time < job->release_time) throw std::invalid_argument("event before job release");
      }
      if (kind == "released") {
        if (states.at(id) != JobState::Unreleased)
          throw std::invalid_argument("duplicate or late release event");
        states.at(id) = JobState::Released;
      } else if (kind == "selected") {
        if (states.at(id) != JobState::Released || !active_job.empty() || loaded_failure)
          throw std::invalid_argument("invalid selection or overlapping deliveries");
        metrics.idle += time - idle_since;
        idle_since = -1;
        active_job = id;
        states.at(id) = JobState::Active;
      } else if (kind == "picked") {
        if (states.at(id) != JobState::Active || active_job != id)
          throw std::invalid_argument("picked before selected or duplicate pickup");
        states.at(id) = JobState::Picked;
      } else if (kind == "delivered") {
        if (states.at(id) != JobState::Picked || active_job != id)
          throw std::invalid_argument("delivered before picked or after terminal job state");
        states.at(id) = JobState::Delivered;
        delivered.emplace(id, time);
        active_job.clear();
        idle_since = time;
      } else if (kind == "failed") {
        if (active_job != id || (states.at(id) != JobState::Active &&
                                states.at(id) != JobState::Picked))
          throw std::invalid_argument("failure outside active delivery");
        loaded_failure = states.at(id) == JobState::Picked;
        if (!loaded_failure) idle_since = time;
        states.at(id) = JobState::Failed;
        failed.insert(id);
        active_job.clear();
      } else if (kind == "decision") {
        ++metrics.decisions;
        if (event[4].rfind("latency_s=", 0) == 0) {
          latencies.push_back(nonnegative_number(event[4].substr(10), "decision latency") * 1000);
        } else {
          complete_latencies = false;
        }
      } else if (terminal_event(kind)) {
        // Время mission ledger относится к началу миссии, runner может начать наблюдение позже.
        terminal_seen = true;
        terminal_kind = kind;
        metrics.elapsed = time;
        if (kind == "episode_completed" && (!active_job.empty() || !failed.empty()))
          throw std::invalid_argument("completed episode contains active or failed delivery");
        if (kind == "episode_completed") {
          for (const auto &[job_id, state] : states) {
            (void)job_id;
            if (state != JobState::Delivered)
              throw std::invalid_argument("completed episode contains unfinished job");
          }
        }
      }
      if (kind == "fallback") ++metrics.fallbacks;
      if (kind == "recovery") ++metrics.recoveries;
    }
    if (previous_time > metrics.elapsed)
      throw std::invalid_argument("ledger extends beyond runner duration without terminal event");
    if (metrics.terminal_reason == "completed" && !terminal_seen)
      throw std::invalid_argument("completed runner has no terminal mission event");
    if (metrics.terminal_reason == "completed" && terminal_kind != "episode_completed")
      throw std::invalid_argument("runner and mission terminal states disagree");
    if (idle_since >= 0) metrics.idle += metrics.elapsed - idle_since;
    if (!latencies.empty()) {
      std::sort(latencies.begin(), latencies.end());
      const auto index = static_cast<std::size_t>(std::ceil(0.95 * latencies.size())) - 1;
      metrics.decision_latency_p95_ms = latencies[index];
    }
    std::vector<double> tardiness;
    double total_age = 0;
    for (const auto &job : scenario.jobs) {
      if (job.release_time > metrics.elapsed) continue;
      ++metrics.released;
      const auto completion = delivered.find(job.id);
      if (completion != delivered.end()) {
        ++metrics.delivered;
        if (completion->second <= job.due_time) ++metrics.on_time;
        tardiness.push_back(std::max(0.0, completion->second - job.due_time));
      } else {
        ++metrics.unfinished;
        total_age += metrics.elapsed - job.release_time;
      }
      if (failed.count(job.id)) ++metrics.failed;
    }
    if (!tardiness.empty()) {
      std::sort(tardiness.begin(), tardiness.end());
      metrics.tardiness_mean = std::accumulate(tardiness.begin(), tardiness.end(), 0.0) / tardiness.size();
      metrics.tardiness_p95 = tardiness[static_cast<std::size_t>(std::ceil(0.95 * tardiness.size())) - 1];
    }
    if (metrics.unfinished) metrics.unfinished_age_mean = total_age / metrics.unfinished;
    // Счётчики, которых нет в ledger, не выдаются за измеренный ноль.
    auto row = parse_csv_row(metrics_row(metrics));
    if (latencies.empty() || !complete_latencies) row[21] = "NA";
    std::ofstream output(argv[4]);
    output << metrics_header() << '\n';
    for (std::size_t i = 0; i < row.size(); ++i) {
      if (i) output << ',';
      output << csv_escape(row[i]);
    }
    output << '\n';
    if (!output) throw std::runtime_error("cannot write metrics");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "mission_metrics: " << error.what() << '\n';
    return 1;
  }
}
