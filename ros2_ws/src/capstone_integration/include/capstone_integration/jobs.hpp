#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "capstone_integration/kinematics.hpp"

namespace capstone_integration {

// One delivery job of the manifest: pick up at `pickup`, drop at `dropoff`.
// `release_s` is relative to the episode start; `due_s` is the delivery budget after release.
struct Job {
  std::string id;
  Pose2D pickup;
  Pose2D dropoff;
  double release_s{0.0};
  double due_s{120.0};
};

// Parses the simple CSV manifest: job_id,px,py,dx,dy,release_s,due_s.
// '#' comments and the "job_id,..." header are skipped. Throws on a malformed line.
std::vector<Job> parse_jobs(const std::string &text);

// Six chained jobs around the warehouse stations (config/jobs.csv); used when no file is given.
std::vector<Job> default_jobs();

// Classic dispatch: the released, not-yet-delivered job with the earliest due time
// (release_s + due_s), ties broken by job id. Returns jobs.size() when nothing is dispatchable.
std::size_t select_next_job(const std::vector<Job> &jobs, const std::vector<bool> &released,
                            const std::vector<bool> &delivered);

}  // namespace capstone_integration
