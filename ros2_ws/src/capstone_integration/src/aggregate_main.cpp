#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "capstone_integration/aggregate.hpp"
#include "capstone_integration/episode_record.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
  const std::string input = argc > 1 ? argv[1] : "results";
  const std::string output = argc > 2 ? argv[2] : (input + "/summary.csv");

  if (!fs::exists(input)) {
    std::cerr << "benchmark_aggregate: no such directory: " << input << '\n';
    return 2;
  }

  std::vector<fs::path> files;
  for (const auto &entry : fs::recursive_directory_iterator(input)) {
    if (entry.is_regular_file() && entry.path().filename() == "episode.csv") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());

  std::vector<capstone_integration::EpisodeRecord> records;
  for (const auto &file : files) {
    std::ifstream in(file);
    if (!in) {
      std::cerr << "benchmark_aggregate: cannot read " << file << '\n';
      return 1;
    }
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
      if (line.empty()) {
        continue;
      }
      if (header) {
        header = false;
        continue;
      }
      try {
        records.push_back(capstone_integration::parse_csv_row(line));
      } catch (const std::exception &error) {
        std::cerr << "benchmark_aggregate: " << file << ": " << error.what() << '\n';
        return 1;
      }
    }
  }

  if (records.empty()) {
    std::cerr << "benchmark_aggregate: no episode rows under " << input << '\n';
    return 1;
  }

  const auto summary = capstone_integration::aggregate(records);
  std::ofstream out(output);
  if (!out) {
    std::cerr << "benchmark_aggregate: cannot write " << output << '\n';
    return 1;
  }
  out << capstone_integration::summary_csv_header() << '\n';
  for (const auto &row : summary) {
    out << capstone_integration::summary_csv_row(row) << '\n';
  }

  std::printf("%-12s %-6s %3s %4s %6s %-14s %4s %4s %8s %8s %7s\n", "CONFIG", "SCEN", "N",
              "OK", "RATE", "95% CI", "TO", "AB", "MEAN_S", "P95_S", "DIST_M");
  for (const auto &row : summary) {
    char interval[32];
    std::snprintf(interval, sizeof(interval), "%.2f..%.2f", row.ci_low, row.ci_high);
    std::printf("%-12s %-6s %3d %4d %6.2f %-14s %4d %4d %8.2f %8.2f %7.2f\n",
                row.config_id.c_str(), row.scenario_id.c_str(), row.episodes, row.success,
                row.success_rate, interval, row.timeout, row.aborted, row.mean_elapsed_s,
                row.p95_elapsed_s, row.mean_distance_m);
  }
  std::cout << "summary written to " << output << " (" << records.size() << " episodes, "
            << summary.size() << " group(s))" << std::endl;
  return 0;
}
