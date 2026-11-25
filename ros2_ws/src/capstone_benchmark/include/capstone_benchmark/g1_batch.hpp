#ifndef CAPSTONE_G1_BATCH_HPP
#define CAPSTONE_G1_BATCH_HPP
#include "capstone_benchmark/g1_eval.hpp"
#include <filesystem>
#include <iosfwd>
#include <random>
#include <tuple>

namespace capstone::g1 {
struct Plan {
  std::string episode,method,scenario,layout,training_seed,evidence,input_id,terminal,format,trace_sha256;
  unsigned scenario_seed{};
  Stamp start{};
  std::filesystem::path trace;
};
struct Episode {
  Plan plan;
  std::string status, reason;
  Coverage coverage;
  std::optional<Summary> summary;
  Stream truth;
};
struct Dist { double mean{}, median{}, sd{}; };
using Group = std::tuple<std::string,std::string,std::string>; // evidence, scenario, method
std::vector<Plan> read_plan(std::istream&);
Episode score_episode(Plan, const std::filesystem::path&);
Dist describe(std::vector<double>);
bool compatible_pair(const Episode&, const Episode&);
std::pair<double,double> bootstrap_mean_ci(const std::vector<double>&, std::mt19937&);
void write_batch(const std::vector<Episode>&,std::ostream&,std::ostream&,std::ostream&,unsigned);
}  // namespace capstone::g1
#endif
