#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "capstone_integration/jobs.hpp"

namespace {

using capstone_integration::default_jobs;
using capstone_integration::Job;
using capstone_integration::parse_jobs;
using capstone_integration::select_next_job;

TEST(ParseJobs, SkipsCommentsAndHeader) {
  const std::string text =
      "# comment\n"
      "job_id,a,b,c,d,e,f\n"
      "j1,1.0,2.0,3.0,4.0,5.0,120.0\n"
      "\n"
      "j2,-1.0,-2.0,-3.0,-4.0,0.0,60.0\n";
  const auto jobs = parse_jobs(text);
  ASSERT_EQ(jobs.size(), 2U);
  EXPECT_EQ(jobs[0].id, "j1");
  EXPECT_DOUBLE_EQ(jobs[0].pickup.x, 1.0);
  EXPECT_DOUBLE_EQ(jobs[0].dropoff.y, 4.0);
  EXPECT_DOUBLE_EQ(jobs[0].release_s, 5.0);
  EXPECT_DOUBLE_EQ(jobs[0].due_s, 120.0);
  EXPECT_EQ(jobs[1].id, "j2");
}

TEST(ParseJobs, RejectsBrokenLines) {
  EXPECT_THROW(parse_jobs("j1,1,2,3\n"), std::invalid_argument);
  EXPECT_THROW(parse_jobs("j1,1,2,3,4,5,abc\n"), std::invalid_argument);
  EXPECT_THROW(parse_jobs("j1,1,2,3,4,-1,120\n"), std::invalid_argument);
  EXPECT_THROW(parse_jobs("j1,1,2,3,4,0,0\n"), std::invalid_argument);
  EXPECT_THROW(parse_jobs("# only comments\n"), std::invalid_argument);
  EXPECT_THROW(parse_jobs(",1,2,3,4,0,120\n"), std::invalid_argument);
}

TEST(DefaultJobs, SixChainedDeliveries) {
  const auto jobs = default_jobs();
  ASSERT_EQ(jobs.size(), 6U);
  EXPECT_EQ(jobs.front().id, "j1");
  EXPECT_DOUBLE_EQ(jobs.front().release_s, 0.0);
  EXPECT_DOUBLE_EQ(jobs.back().release_s, 160.0);
  // The drop-off of one job is the pick-up of the next one.
  for (std::size_t i = 0; i + 1 < jobs.size(); ++i) {
    EXPECT_NEAR(jobs[i].dropoff.x, jobs[i + 1].pickup.x, 1e-9);
    EXPECT_NEAR(jobs[i].dropoff.y, jobs[i + 1].pickup.y, 1e-9);
  }
}

TEST(SelectNextJob, EarliestDueAmongReleased) {
  const auto jobs = default_jobs();
  std::vector<bool> released(jobs.size(), false);
  std::vector<bool> delivered(jobs.size(), false);

  EXPECT_EQ(select_next_job(jobs, released, delivered), jobs.size());

  released[0] = true;
  released[1] = true;
  EXPECT_EQ(select_next_job(jobs, released, delivered), 0U);

  delivered[0] = true;
  EXPECT_EQ(select_next_job(jobs, released, delivered), 1U);
}

TEST(SelectNextJob, TiesAreBrokenByJobId) {
  const std::vector<Job> jobs = {
      {"b", {0, 0, 0}, {0, 0, 0}, 0.0, 100.0},
      {"a", {0, 0, 0}, {0, 0, 0}, 0.0, 100.0},
  };
  const std::vector<bool> released = {true, true};
  const std::vector<bool> delivered = {false, false};
  EXPECT_EQ(select_next_job(jobs, released, delivered), 1U);
}

TEST(SelectNextJob, RejectsMismatchedBookkeeping) {
  const auto jobs = default_jobs();
  EXPECT_THROW(select_next_job(jobs, {true}, {false}), std::invalid_argument);
}

}  // namespace
