#include "capstone_mission/domain.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <tuple>
#include <yaml-cpp/yaml.h>

namespace capstone_mission {
namespace {
void require(bool condition, const std::string &message) {
  if (!condition) throw std::invalid_argument(message);
}
bool nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
void validate_matrix(const std::vector<std::vector<double>> &matrix, std::size_t count,
                     const std::string &name) {
  if (matrix.empty()) return;
  require(matrix.size() == count, name + ": invalid row count");
  for (const auto &row : matrix) {
    require(row.size() == count, name + ": invalid column count");
    for (double value : row) require(nonnegative(value), name + ": invalid value");
  }
}
void validate_job(const Job &job) {
  require(!job.id.empty(), "empty job id");
  require(nonnegative(job.release_time) && nonnegative(job.due_time), "invalid job time");
  require(nonnegative(job.pickup_service) && nonnegative(job.dropoff_service),
          "invalid service duration");
}
void validate_dispatch(const std::vector<Job *> &jobs,
                       const std::vector<TravelEstimate> &travel) {
  require(jobs.size() <= 4 && jobs.size() == travel.size(), "invalid candidate set");
  for (std::size_t i = 0; i < jobs.size(); ++i) {
    require(jobs[i] != nullptr && jobs[i]->state == JobState::Released,
            "candidate must be a released job");
    const auto &estimate = travel[i];
    require(nonnegative(estimate.pickup_time) && nonnegative(estimate.delivery_time) &&
            nonnegative(estimate.route_length) && nonnegative(estimate.congestion),
            "invalid travel estimate");
  }
}
}  // namespace

std::string to_string(JobState state) {
  switch (state) {
    case JobState::Unreleased: return "unreleased";
    case JobState::Released: return "released";
    case JobState::Active: return "active";
    case JobState::Picked: return "picked";
    case JobState::Delivered: return "delivered";
    case JobState::Failed: return "failed";
  }
  throw std::invalid_argument("invalid job state");
}

std::size_t station_index(const Scenario &scenario, const std::string &id) {
  for (std::size_t i = 0; i < scenario.stations.size(); ++i) {
    if (scenario.stations[i].id == id) return i;
  }
  throw std::invalid_argument("unknown station: " + id);
}
const Station &station_by_id(const Scenario &scenario, const std::string &id) {
  return scenario.stations.at(station_index(scenario, id));
}

void validate_scenario(const Scenario &scenario) {
  require(!scenario.episode_id.empty(), "empty episode id");
  require(std::isfinite(scenario.horizon) && scenario.horizon > 0.0, "invalid horizon");
  require(!scenario.stations.empty(), "scenario has no stations");
  std::set<std::string> station_ids;
  for (const auto &station : scenario.stations) {
    require(!station.id.empty() && station_ids.insert(station.id).second,
            "duplicate or empty station id");
    require(std::isfinite(station.x) && std::isfinite(station.y) &&
            std::isfinite(station.yaw), "invalid station pose");
  }
  station_by_id(scenario, scenario.initial_station);
  std::set<std::string> job_ids;
  for (const auto &job : scenario.jobs) {
    validate_job(job);
    require(job_ids.insert(job.id).second, "duplicate job id: " + job.id);
    station_by_id(scenario, job.pickup);
    station_by_id(scenario, job.dropoff);
  }
  validate_matrix(scenario.travel_times, scenario.stations.size(), "travel_times");
  validate_matrix(scenario.congestion, scenario.stations.size(), "congestion");
  if (!scenario.blocked.empty()) {
    require(scenario.blocked.size() == scenario.stations.size(), "invalid blocked rows");
    for (const auto &row : scenario.blocked)
      require(row.size() == scenario.stations.size(), "invalid blocked columns");
  }
}

Scenario load_scenario(const std::string &path) {
  const auto root = YAML::LoadFile(path);
  Scenario scenario;
  scenario.episode_id = root["episode_id"].as<std::string>();
  scenario.split = root["split"].as<std::string>("debug");
  scenario.family = root["family"].as<std::string>("low");
  scenario.seed = root["scenario_seed"].as<int>(root["seed"].as<int>(0));
  scenario.layout_id = root["layout_id"].as<std::string>("debug");
  scenario.horizon = root["horizon"].as<double>(300.0);
  scenario.initial_station = root["initial_station"].as<std::string>();
  for (const auto &entry : root["stations"]) {
    scenario.stations.push_back({entry["id"].as<std::string>(), entry["x"].as<double>(),
                                entry["y"].as<double>(), entry["yaw"].as<double>(0.0)});
  }
  for (const auto &entry : root["jobs"]) {
    Job job;
    job.id = entry["id"].as<std::string>();
    job.pickup = entry["pickup"].as<std::string>();
    job.dropoff = entry["dropoff"].as<std::string>();
    job.release_time = entry["release_time"].as<double>();
    job.due_time = entry["due_time"].as<double>();
    job.pickup_service = entry["pickup_service"].as<double>(1.0);
    job.dropoff_service = entry["dropoff_service"].as<double>(1.0);
    scenario.jobs.push_back(job);
  }
  if (root["travel_times"])
    scenario.travel_times = root["travel_times"].as<std::vector<std::vector<double>>>();
  if (root["congestion"])
    scenario.congestion = root["congestion"].as<std::vector<std::vector<double>>>();
  if (root["blocked"])
    scenario.blocked = root["blocked"].as<std::vector<std::vector<bool>>>();
  validate_scenario(scenario);
  return scenario;
}

JobQueue::JobQueue(std::vector<Job> jobs) : jobs_(std::move(jobs)) {
  std::set<std::string> ids;
  for (const auto &job : jobs_) {
    validate_job(job);
    require(ids.insert(job.id).second, "duplicate job id: " + job.id);
  }
  reset();
}
void JobQueue::reset() {
  for (auto &job : jobs_) {
    job.state = JobState::Unreleased;
    job.picked_time = -1.0;
    job.delivered_time = -1.0;
    job.failure_reason.clear();
  }
}
void JobQueue::release(double now) {
  require(nonnegative(now), "invalid simulation time");
  for (auto &job : jobs_) {
    if (job.state == JobState::Unreleased && job.release_time <= now)
      job.state = JobState::Released;
  }
}
std::vector<Job *> JobQueue::candidates() {
  std::vector<Job *> result;
  for (auto &job : jobs_) if (job.state == JobState::Released) result.push_back(&job);
  std::sort(result.begin(), result.end(), [](const Job *left, const Job *right) {
    return std::tie(left->release_time, left->id) < std::tie(right->release_time, right->id);
  });
  if (result.size() > 4) result.resize(4);
  return result;
}
Job &JobQueue::find(const std::string &id) {
  for (auto &job : jobs_) if (job.id == id) return job;
  throw std::invalid_argument("unknown job id: " + id);
}
void JobQueue::mark_active(const std::string &id) {
  auto &job = find(id);
  require(job.state == JobState::Released, "job is not available");
  for (const auto &other : jobs_)
    require(other.state != JobState::Active && other.state != JobState::Picked,
            "another delivery is active");
  job.state = JobState::Active;
}
void JobQueue::mark_picked(const std::string &id, double now) {
  auto &job = find(id);
  require(job.state == JobState::Active && nonnegative(now) && now >= job.release_time,
          "invalid pickup transition");
  job.state = JobState::Picked;
  job.picked_time = now;
}
void JobQueue::mark_delivered(const std::string &id, double now) {
  auto &job = find(id);
  require(job.state == JobState::Picked && std::isfinite(now) && now >= job.picked_time,
          "invalid delivery transition");
  job.state = JobState::Delivered;
  job.delivered_time = now;
}
void JobQueue::mark_failed(const std::string &id, const std::string &reason) {
  auto &job = find(id);
  require(job.state == JobState::Active || job.state == JobState::Picked ||
          job.state == JobState::Released, "invalid failure transition");
  require(!reason.empty(), "failure requires a reason");
  job.state = JobState::Failed;
  job.failure_reason = reason;
}
bool JobQueue::all_terminal() const {
  return std::all_of(jobs_.begin(), jobs_.end(), [](const Job &job) {
    return job.state == JobState::Delivered || job.state == JobState::Failed;
  });
}

std::vector<TravelEstimate> estimate_travel(
    const Scenario &scenario, const std::vector<Job *> &candidates,
    const DispatchContext &context, const std::string &current_station) {
  std::vector<TravelEstimate> result;
  for (const Job *job : candidates) {
    require(job != nullptr, "null job");
    const auto from = station_index(scenario, job->pickup);
    const auto to = station_index(scenario, job->dropoff);
    const auto &pickup = scenario.stations[from];
    const auto &dropoff = scenario.stations[to];
    double pickup_base = std::hypot(pickup.x - context.robot_x, pickup.y - context.robot_y) / 0.5;
    double delivery_base = std::hypot(dropoff.x - pickup.x, dropoff.y - pickup.y) / 0.5;
    double pickup_factor = 1.0;
    double delivery_factor = 1.0;
    if (!scenario.travel_times.empty()) {
      delivery_base = scenario.travel_times.at(from).at(to);
      if (!current_station.empty())
        pickup_base = scenario.travel_times.at(station_index(scenario, current_station)).at(from);
    }
    if (!scenario.congestion.empty()) {
      delivery_factor = scenario.congestion.at(from).at(to);
      if (!current_station.empty())
        pickup_factor = scenario.congestion.at(station_index(scenario, current_station)).at(from);
    }
    result.push_back({pickup_base * pickup_factor, delivery_base * delivery_factor,
                      (pickup_base + delivery_base) * 0.5, delivery_factor});
  }
  return result;
}

Observation make_observation(const std::vector<Job *> &candidates,
                             const std::vector<TravelEstimate> &travel,
                             const DispatchContext &context, bool congestion_feature) {
  validate_dispatch(candidates, travel);
  require(nonnegative(context.now) && std::isfinite(context.robot_x) &&
          std::isfinite(context.robot_y) && context.navigation_failures >= 0,
          "invalid dispatch context");
  Observation observation{};
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Job &job = *candidates[i];
    const auto &estimate = travel[i];
    require(job.release_time <= context.now, "future job in candidate set");
    const double slack = job.due_time - context.now - estimate.pickup_time -
                         estimate.delivery_time - job.pickup_service - job.dropoff_service;
    observation[i * 7] = static_cast<float>(estimate.pickup_time / 120.0);
    observation[i * 7 + 1] = static_cast<float>(estimate.delivery_time / 120.0);
    observation[i * 7 + 2] = static_cast<float>(slack / 300.0);
    observation[i * 7 + 3] = static_cast<float>((context.now - job.release_time) / 300.0);
    observation[i * 7 + 4] = congestion_feature ? static_cast<float>(estimate.congestion / 3.0) : 0.0F;
    observation[i * 7 + 5] = static_cast<float>(context.navigation_failures / 2.0);
    observation[i * 7 + 6] = 1.0F;
  }
  observation[28] = static_cast<float>(context.robot_x / 20.0);
  observation[29] = static_cast<float>(context.robot_y / 20.0);
  observation[30] = context.loaded ? 1.0F : 0.0F;
  observation[31] = static_cast<float>(context.now / 300.0);
  return observation;
}
ActionMask make_action_mask(const std::vector<Job *> &candidates, bool loaded) {
  require(candidates.size() <= 4, "too many candidates");
  ActionMask mask{false, false, false, false, true};
  if (!loaded) for (std::size_t i = 0; i < candidates.size(); ++i)
    mask[i] = candidates[i] && candidates[i]->state == JobState::Released;
  return mask;
}
int classical_action(const std::vector<Job *> &candidates,
                     const std::vector<TravelEstimate> &travel) {
  validate_dispatch(candidates, travel);
  if (candidates.empty()) return 4;
  std::size_t best = 0;
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    if (std::tie(candidates[i]->due_time, travel[i].route_length, candidates[i]->id) <
        std::tie(candidates[best]->due_time, travel[best].route_length, candidates[best]->id)) best = i;
  }
  return static_cast<int>(best);
}
}  // namespace capstone_mission
