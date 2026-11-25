#include "capstone_fleet_runtime/fleet_jobs.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capstone_fleet_runtime {
namespace {
using capstone_mission::JobState;
void require(bool condition, const std::string &reason) {
  if (!condition) throw std::invalid_argument(reason);
}
bool active(JobState state) { return state == JobState::Active || state == JobState::Picked; }
}  // namespace

FleetJobQueue::FleetJobQueue(std::vector<capstone_mission::Job> jobs) {
  // Общая валидация формата Job, но дальнейшие переходы принадлежат очереди флота.
  const capstone_mission::JobQueue validated(std::move(jobs));
  jobs_ = validated.jobs();
}

void FleetJobQueue::release(double now) {
  require(std::isfinite(now) && now >= 0, "invalid simulation time");
  for (auto &job : jobs_) {
    if (job.state == JobState::Unreleased && job.release_time <= now) job.state = JobState::Released;
  }
}

const capstone_mission::Job &FleetJobQueue::find(const std::string &id) const {
  for (const auto &job : jobs_) if (job.id == id) return job;
  throw std::invalid_argument("unknown job id: " + id);
}

capstone_mission::Job &FleetJobQueue::mutable_job(const std::string &id) {
  for (auto &job : jobs_) if (job.id == id) return job;
  throw std::invalid_argument("unknown job id: " + id);
}

std::string FleetJobQueue::owner(const std::string &id) const {
  find(id);
  const auto owner = owners_.find(id);
  return owner == owners_.end() ? "" : owner->second;
}

void FleetJobQueue::require_owner(const std::string &id, const std::string &robot) const {
  require(!robot.empty() && owner(id) == robot, "job belongs to another robot: " + id);
}

void FleetJobQueue::assign(const std::string &id, const std::string &robot) {
  auto &job = mutable_job(id);
  require(!robot.empty(), "empty robot id");
  require(job.state == JobState::Released && owners_.count(id) == 0, "job is not available: " + id);
  for (const auto &other : jobs_) {
    require(!active(other.state) || owner(other.id) != robot, "robot already owns a delivery: " + robot);
  }
  owners_[id] = robot;
  job.state = JobState::Active;
}

void FleetJobQueue::mark_picked(const std::string &id, const std::string &robot, double now) {
  require_owner(id, robot);
  auto &job = mutable_job(id);
  require(job.state == JobState::Active && std::isfinite(now) && now >= job.release_time,
          "invalid pickup transition: " + id);
  job.state = JobState::Picked;
  job.picked_time = now;
}

void FleetJobQueue::mark_delivered(const std::string &id, const std::string &robot, double now) {
  require_owner(id, robot);
  auto &job = mutable_job(id);
  require(job.state == JobState::Picked && std::isfinite(now) && now >= job.picked_time,
          "invalid delivery transition: " + id);
  job.state = JobState::Delivered;
  job.delivered_time = now;
}

void FleetJobQueue::mark_failed(const std::string &id, const std::string &robot,
                                const std::string &reason) {
  require_owner(id, robot);
  auto &job = mutable_job(id);
  require(active(job.state) && !reason.empty(), "invalid failure transition: " + id);
  job.state = JobState::Failed;
  job.failure_reason = reason;
}

bool FleetJobQueue::all_terminal() const {
  return std::all_of(jobs_.begin(), jobs_.end(), [](const auto &job) {
    return job.state == JobState::Delivered || job.state == JobState::Failed;
  });
}
}  // namespace capstone_fleet_runtime
