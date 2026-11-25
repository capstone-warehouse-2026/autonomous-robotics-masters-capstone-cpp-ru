#include "capstone_integration/jobs.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace capstone_integration {

namespace {

Job parse_line(const std::string &line) {
  std::vector<std::string> fields;
  std::string current;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  fields.push_back(current);
  if (fields.size() != 7) {
    throw std::invalid_argument("parse_jobs: expected 7 fields, got " +
                                std::to_string(fields.size()) + " in '" + line + "'");
  }
  Job job;
  try {
    job.id = fields[0];
    job.pickup.x = std::stod(fields[1]);
    job.pickup.y = std::stod(fields[2]);
    job.dropoff.x = std::stod(fields[3]);
    job.dropoff.y = std::stod(fields[4]);
    job.release_s = std::stod(fields[5]);
    job.due_s = std::stod(fields[6]);
  } catch (const std::exception &) {
    throw std::invalid_argument("parse_jobs: a numeric field is not a number in '" + line + "'");
  }
  if (job.id.empty()) {
    throw std::invalid_argument("parse_jobs: empty job id");
  }
  if (job.release_s < 0.0 || !(job.due_s > 0.0)) {
    throw std::invalid_argument("parse_jobs: release_s >= 0 and due_s > 0 required");
  }
  if (!std::isfinite(job.pickup.x) || !std::isfinite(job.pickup.y) ||
      !std::isfinite(job.dropoff.x) || !std::isfinite(job.dropoff.y)) {
    throw std::invalid_argument("parse_jobs: non-finite station coordinates");
  }
  // Drop-off yaw faces the row of stations; the placeholder does not need a precise heading.
  job.pickup.yaw = 0.0;
  job.dropoff.yaw = 0.0;
  return job;
}

}  // namespace

std::vector<Job> parse_jobs(const std::string &text) {
  std::vector<Job> jobs;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line.rfind("job_id", 0) == 0) {
      continue;
    }
    jobs.push_back(parse_line(line));
  }
  if (jobs.empty()) {
    throw std::invalid_argument("parse_jobs: no jobs in the manifest");
  }
  return jobs;
}

std::vector<Job> default_jobs() {
  return parse_jobs(
      "job_id,pickup_x,pickup_y,dropoff_x,dropoff_y,release_s,due_s\n"
      "j1,-8.8,7.5,-1.0,8.8,0,120\n"
      "j2,-1.0,8.8,8.8,7.5,20,120\n"
      "j3,8.8,7.5,8.8,-7.5,50,120\n"
      "j4,8.8,-7.5,-1.0,-8.8,80,120\n"
      "j5,-1.0,-8.8,-8.8,-7.5,120,120\n"
      "j6,-8.8,-7.5,-8.8,7.5,160,120\n");
}

std::size_t select_next_job(const std::vector<Job> &jobs, const std::vector<bool> &released,
                            const std::vector<bool> &delivered) {
  if (jobs.size() != released.size() || jobs.size() != delivered.size()) {
    throw std::invalid_argument("select_next_job: bookkeeping vectors must match the job list");
  }
  std::size_t best = jobs.size();
  double best_due = 0.0;
  for (std::size_t i = 0; i < jobs.size(); ++i) {
    if (!released[i] || delivered[i]) {
      continue;
    }
    const double due = jobs[i].release_s + jobs[i].due_s;
    if (best == jobs.size() || due < best_due ||
        (due == best_due && jobs[i].id < jobs[best].id)) {
      best = i;
      best_due = due;
    }
  }
  return best;
}

}  // namespace capstone_integration
