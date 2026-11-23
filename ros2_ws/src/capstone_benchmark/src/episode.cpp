#include "capstone_benchmark/episode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>

namespace capstone_benchmark {
using namespace capstone_mission;
namespace {
double percentile(std::vector<double> values, double probability) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(std::ceil(probability * values.size())) - 1;
  return values[std::min(index, values.size() - 1)];
}
}

double Metrics::throughput() const { return elapsed > 0 ? delivered * 3600.0 / elapsed : 0; }
double Metrics::on_time_fraction() const {
  return released > 0 ? static_cast<double>(on_time) / released : 0;
}

EpisodeResult run_episode(const Scenario &scenario, const std::string &method,
                          const DqnModel *model, bool congestion_feature) {
  validate_scenario(scenario);
  const bool learned = method != "classical";
  JobQueue queue(scenario.jobs);
  EpisodeResult output;
  auto &metrics = output.metrics;
  metrics.episode_id = scenario.episode_id;
  metrics.family = scenario.family;
  metrics.method = method;
  std::string current_station = scenario.initial_station;
  double now = 0;
  int navigation_failures = 0;
  std::vector<double> latencies;
  auto record = [&](const Job &job, const std::string &event, double time,
                    const std::string &reason = "", const std::string &target = "") {
    output.events.push_back({scenario.episode_id, job.id, time, event, reason, target});
  };
  auto release_until = [&](double time) {
    for (const auto &job : queue.jobs()) {
      if (job.state == JobState::Unreleased && job.release_time <= time) {
        record(job, "released", job.release_time);
      }
    }
    queue.release(time);
  };
  auto advance = [&](double duration, double distance = 0.0) {
    const double actual = std::min(duration, scenario.horizon - now);
    if (duration > 0) metrics.distance += distance * actual / duration;
    now += actual;
    release_until(now);
    return actual >= duration;
  };
  release_until(now);
  while (now < scenario.horizon && !queue.all_terminal()) {
    const auto candidates = queue.candidates();
    const auto &robot = station_by_id(scenario, current_station);
    DispatchContext context{now, robot.x, robot.y, false, navigation_failures};
    const auto travel = estimate_travel(scenario, candidates, context, current_station);
    const auto started = std::chrono::steady_clock::now();
    const auto decision = choose_action(candidates, travel, context, model, learned,
                                        congestion_feature);
    latencies.push_back(std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - started).count());
    ++metrics.decisions;
    if (decision.fallback) ++metrics.fallbacks;
    if (decision.action == 4) {
      const double wait = std::min(1.0, scenario.horizon - now);
      metrics.idle += wait;
      output.events.push_back({scenario.episode_id, "", now, "wait", decision.reason, ""});
      advance(wait);
      continue;
    }
    Job &job = *candidates.at(static_cast<std::size_t>(decision.action));
    const auto &estimate = travel.at(static_cast<std::size_t>(decision.action));
    queue.mark_active(job.id);
    record(job, "selected", now, decision.reason, job.pickup);
    // В табличной среде длина маршрута получена из номинального времени при 0.5 м/с.
    const auto from = station_index(scenario, current_station);
    const auto pickup = station_index(scenario, job.pickup);
    const auto dropoff = station_index(scenario, job.dropoff);
    const double pickup_distance = scenario.travel_times.empty() ? estimate.pickup_time * 0.5 :
                                   scenario.travel_times[from][pickup] * 0.5;
    const double delivery_distance = scenario.travel_times.empty() ? estimate.delivery_time * 0.5 :
                                     scenario.travel_times[pickup][dropoff] * 0.5;
    if (!scenario.blocked.empty() && scenario.blocked[from][pickup]) {
      advance(2.0);
      ++metrics.recoveries;
      navigation_failures += 2;
      queue.mark_failed(job.id, "pickup_no_path");
      record(job, "failed", now, "pickup_no_path", job.pickup);
      continue;
    }
    if (!advance(estimate.pickup_time, pickup_distance)) break;
    current_station = job.pickup;
    if (!advance(job.pickup_service)) break;
    queue.mark_picked(job.id, now);
    record(job, "picked", now, "", job.pickup);
    if (!scenario.blocked.empty() && scenario.blocked[pickup][dropoff]) {
      advance(2.0);
      ++metrics.recoveries;
      navigation_failures += 2;
      queue.mark_failed(job.id, "dropoff_no_path_loaded");
      record(job, "failed", now, "dropoff_no_path_loaded", job.dropoff);
      // Робот остаётся с грузом: следующие заявки не выбираются, но не исчезают из учёта.
      advance(scenario.horizon - now);
      break;
    }
    if (!advance(estimate.delivery_time, delivery_distance)) break;
    current_station = job.dropoff;
    if (!advance(job.dropoff_service)) break;
    queue.mark_delivered(job.id, now);
    record(job, "delivered", now, "", job.dropoff);
  }
  metrics.elapsed = now;
  std::vector<double> tardiness;
  double total_age = 0;
  for (const auto &job : queue.jobs()) {
    if (job.state == JobState::Unreleased) continue;
    ++metrics.released;
    if (job.state == JobState::Delivered) {
      ++metrics.delivered;
      if (job.delivered_time <= job.due_time) ++metrics.on_time;
      tardiness.push_back(std::max(0.0, job.delivered_time - job.due_time));
    } else {
      ++metrics.unfinished;
      total_age += now - job.release_time;
      record(job, "unfinished", now, "episode_end");
    }
    if (job.state == JobState::Failed) ++metrics.failed;
  }
  if (!tardiness.empty()) {
    metrics.tardiness_mean = std::accumulate(tardiness.begin(), tardiness.end(), 0.0) / tardiness.size();
    metrics.tardiness_p95 = percentile(tardiness, 0.95);
  }
  if (metrics.unfinished > 0) metrics.unfinished_age_mean = total_age / metrics.unfinished;
  metrics.decision_latency_p95_ms = percentile(latencies, 0.95);
  metrics.terminal_reason = metrics.unfinished == 0 ? "completed" : "timeout";
  if (metrics.failed > 0) metrics.terminal_reason = "navigation_failure";
  std::stable_sort(output.events.begin(), output.events.end(),
                   [](const Event &a, const Event &b) { return a.sim_time < b.sim_time; });
  return output;
}

std::string csv_escape(const std::string &value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
  std::string result = "\"";
  for (char c : value) { if (c == '"') result += '"'; result += c; }
  return result + '"';
}

std::vector<std::string> parse_csv_row(const std::string &line) {
  std::vector<std::string> result;
  std::string value;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') { value += c; ++i; }
      else quoted = !quoted;
    } else if (c == ',' && !quoted) { result.push_back(value); value.clear(); }
    else value += c;
  }
  if (quoted) throw std::invalid_argument("unterminated quoted CSV field");
  result.push_back(value);
  return result;
}

std::string metrics_header() {
  return "episode_id,family,method,terminal_reason,elapsed_sim_s,released,delivered,on_time,"
         "failed,unfinished,unfinished_age_mean_s,deliveries_per_hour,on_time_fraction,"
         "tardiness_mean_s,tardiness_p95_s,distance_m,distance_per_job_m,idle_s,recoveries,"
         "fallbacks,decisions,decision_latency_p95_ms";
}

std::string metrics_row(const Metrics &m) {
  std::ostringstream row;
  row << std::setprecision(12) << csv_escape(m.episode_id) << ',' << csv_escape(m.family) << ','
      << csv_escape(m.method) << ',' << m.terminal_reason << ',' << m.elapsed << ',' << m.released
      << ',' << m.delivered << ',' << m.on_time << ',' << m.failed << ',' << m.unfinished << ','
      << m.unfinished_age_mean << ',' << m.throughput() << ',';
  if (m.released) row << m.on_time_fraction(); else row << "NA";
  row << ',';
  if (m.delivered) row << m.tardiness_mean << ',' << m.tardiness_p95;
  else row << "NA,NA";
  row << ',' << m.distance << ',';
  if (m.delivered) row << m.distance / m.delivered; else row << "NA";
  row << ',' << m.idle << ',' << m.recoveries << ',' << m.fallbacks << ',' << m.decisions
      << ',' << m.decision_latency_p95_ms;
  return row.str();
}

std::pair<double, double> bootstrap_mean_ci(const std::vector<double> &values,
                                           unsigned seed, int samples) {
  if (values.empty() || samples < 1) throw std::invalid_argument("bootstrap needs observations");
  std::mt19937 engine(seed);
  std::uniform_int_distribution<std::size_t> pick(0, values.size() - 1);
  std::vector<double> means;
  means.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    double sum = 0;
    for (std::size_t j = 0; j < values.size(); ++j) sum += values[pick(engine)];
    means.push_back(sum / values.size());
  }
  return {percentile(means, 0.025), percentile(means, 0.975)};
}
}  // namespace capstone_benchmark
