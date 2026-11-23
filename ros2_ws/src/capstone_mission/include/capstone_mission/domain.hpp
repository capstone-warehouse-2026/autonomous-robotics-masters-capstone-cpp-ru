#pragma once

#include <array>
#include <string>
#include <vector>

namespace capstone_mission {

enum class JobState { Unreleased, Released, Active, Picked, Delivered, Failed };
std::string to_string(JobState state);

struct Station {
  std::string id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Job {
  std::string id;
  std::string pickup;
  std::string dropoff;
  double release_time{0.0};
  double due_time{0.0};
  double pickup_service{1.0};
  double dropoff_service{1.0};
  JobState state{JobState::Unreleased};
  double picked_time{-1.0};
  double delivered_time{-1.0};
  std::string failure_reason;
};

struct Scenario {
  std::string episode_id{"episode"};
  std::string split{"debug"};
  std::string family{"low"};
  std::string layout_id{"debug"};
  int seed{0};
  double horizon{300.0};
  std::string initial_station;
  std::vector<Station> stations;
  std::vector<Job> jobs;
  std::vector<std::vector<double>> travel_times;
  std::vector<std::vector<double>> congestion;
  std::vector<std::vector<bool>> blocked;
};

Scenario load_scenario(const std::string &path);
void validate_scenario(const Scenario &scenario);
const Station &station_by_id(const Scenario &scenario, const std::string &id);
std::size_t station_index(const Scenario &scenario, const std::string &id);

// Очередь хранит все заявки; политика видит только четыре уже поступившие.
class JobQueue {
 public:
  explicit JobQueue(std::vector<Job> jobs = {});
  void reset();
  void release(double now);
  std::vector<Job *> candidates();
  Job &find(const std::string &id);
  const std::vector<Job> &jobs() const { return jobs_; }
  void mark_active(const std::string &id);
  void mark_picked(const std::string &id, double now);
  void mark_delivered(const std::string &id, double now);
  void mark_failed(const std::string &id, const std::string &reason);
  bool all_terminal() const;
 private:
  std::vector<Job> jobs_;
};

struct TravelEstimate {
  double pickup_time{0.0};
  double delivery_time{0.0};
  double route_length{0.0};
  double congestion{1.0};
};
struct DispatchContext {
  double now{0.0};
  double robot_x{0.0};
  double robot_y{0.0};
  bool loaded{false};
  int navigation_failures{0};
};

// Таблицы допустимы в изолированном эксперименте; геометрическая оценка — в ROS.
std::vector<TravelEstimate> estimate_travel(
    const Scenario &scenario, const std::vector<Job *> &candidates,
    const DispatchContext &context, const std::string &current_station = "");
using Observation = std::array<float, 32>;
using ActionMask = std::array<bool, 5>;
Observation make_observation(const std::vector<Job *> &candidates,
                             const std::vector<TravelEstimate> &travel,
                             const DispatchContext &context, bool congestion_feature = true);
ActionMask make_action_mask(const std::vector<Job *> &candidates, bool loaded = false);
int classical_action(const std::vector<Job *> &candidates,
                     const std::vector<TravelEstimate> &travel);

}  // namespace capstone_mission
