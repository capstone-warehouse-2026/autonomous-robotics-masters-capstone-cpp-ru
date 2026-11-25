#pragma once

#include <map>
#include <string>
#include <vector>
#include "capstone_mission/domain.hpp"

namespace capstone_fleet_runtime {

// Одна активная заявка на каждого робота. Глобальное ограничение одиночного G5
// остаётся в его JobQueue и не меняется для многороботного исполнения.
class FleetJobQueue {
 public:
  explicit FleetJobQueue(std::vector<capstone_mission::Job> jobs = {});
  void release(double now);
  const capstone_mission::Job &find(const std::string &id) const;
  const std::vector<capstone_mission::Job> &jobs() const { return jobs_; }
  std::string owner(const std::string &id) const;
  void assign(const std::string &id, const std::string &robot);
  void mark_picked(const std::string &id, const std::string &robot, double now);
  void mark_delivered(const std::string &id, const std::string &robot, double now);
  void mark_failed(const std::string &id, const std::string &robot, const std::string &reason);
  bool all_terminal() const;

 private:
  capstone_mission::Job &mutable_job(const std::string &id);
  void require_owner(const std::string &id, const std::string &robot) const;
  std::vector<capstone_mission::Job> jobs_;
  std::map<std::string, std::string> owners_;
};
}  // namespace capstone_fleet_runtime
