// Единица статистики — эпизод. Парные разности сохраняют одинаковые scenario IDs.
#include "capstone_benchmark/episode.hpp"
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>

using namespace capstone_benchmark;
int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("usage: aggregate_dispatch metrics.csv summary.csv");
    std::ifstream input(argv[1]);
    if (!input) throw std::runtime_error("cannot read metrics");
    std::string line;
    std::getline(input, line);
    if (line != metrics_header()) throw std::invalid_argument("unexpected metrics schema");
    using Episodes = std::map<std::string, double>;
    using Groups = std::map<std::string, std::map<std::string, Episodes>>;
    Groups groups;
    std::map<std::string, Groups> all_metrics;
    const auto columns = parse_csv_row(metrics_header());
    while (std::getline(input, line)) {
      const auto fields = parse_csv_row(line);
      if (fields.size() != 22) throw std::invalid_argument("invalid metrics row");
      if (fields[12] == "NA") throw std::invalid_argument("on-time fraction undefined for empty episode");
      auto &episodes = groups[fields[1]][fields[2]];
      const double value = std::stod(fields[12]);
      if (!std::isfinite(value) || value < 0 || value > 1) {
        throw std::invalid_argument("invalid on-time fraction");
      }
      if (!episodes.emplace(fields[0], value).second) {
        throw std::invalid_argument("duplicate episode/method");
      }
      for (std::size_t column = 4; column < fields.size(); ++column) {
        auto &samples = all_metrics[columns[column]][fields[1]][fields[2]];
        if (fields[column] == "NA") continue;
        std::size_t consumed = 0;
        const double number = std::stod(fields[column], &consumed);
        if (consumed != fields[column].size() || !std::isfinite(number) || number < 0)
          throw std::invalid_argument("invalid numeric metric: " + columns[column]);
        samples.emplace(fields[0], number);
      }
    }
    const std::set<std::string> expected_families{"low", "high", "bottleneck", "blocked", "unequal"};
    std::set<std::string> actual_families;
    std::set<std::string> expected_methods;
    for (const auto &[family, methods] : groups) {
      actual_families.insert(family);
      std::set<std::string> actual_methods;
      for (const auto &[method, episodes] : methods) {
        actual_methods.insert(method);
        if (episodes.size() != 20) throw std::invalid_argument("expected 20 episodes per family/method");
      }
      if (expected_methods.empty()) expected_methods = actual_methods;
      if (expected_methods != actual_methods) throw std::invalid_argument("incomplete method matrix");
    }
    if (actual_families != expected_families) throw std::invalid_argument("incomplete scenario family matrix");
    std::ofstream output(argv[2]);
    if (!output) throw std::runtime_error("cannot write summary");
    output << "family,method,n,on_time_mean,ci95_low,ci95_high,paired_difference,paired_ci95_low,paired_ci95_high\n";
    for (const auto &[family, methods] : groups) {
      const auto baseline = methods.find("classical");
      if (baseline == methods.end()) throw std::invalid_argument("missing classical baseline");
      for (const auto &[method, episodes] : methods) {
        if (episodes.size() != baseline->second.size()) throw std::invalid_argument("incomplete episode pairing");
        std::vector<double> values, differences;
        for (const auto &[id, value] : episodes) {
          const auto paired = baseline->second.find(id);
          if (paired == baseline->second.end()) throw std::invalid_argument("missing paired episode " + id);
          values.push_back(value);
          differences.push_back(value - paired->second);
        }
        const auto ci = bootstrap_mean_ci(values);
        const auto diff_ci = bootstrap_mean_ci(differences);
        const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        const double difference = std::accumulate(differences.begin(), differences.end(), 0.0) / differences.size();
        output << csv_escape(family) << ',' << csv_escape(method) << ',' << values.size() << ','
               << mean << ',' << ci.first << ',' << ci.second << ',' << difference << ','
               << diff_ci.first << ',' << diff_ci.second << '\n';
      }
    }
    if (!output) throw std::runtime_error("failed to write summary");
    // NA не превращается в ноль: отдельно показываем n валидных и n парных наблюдений.
    const std::filesystem::path summary_path(argv[2]);
    const auto all_path = summary_path.parent_path() /
                          (summary_path.stem().string() + "_all_metrics.csv");
    std::ofstream all_output(all_path);
    if (!all_output) throw std::runtime_error("cannot write all-metric summary");
    all_output << "metric,family,method,n_total,n_valid,mean,ci95_low,ci95_high,"
                  "n_paired,paired_difference,paired_ci95_low,paired_ci95_high\n";
    auto write_estimate = [&all_output](const std::vector<double> &values) {
      if (values.empty()) { all_output << "NA,NA,NA"; return; }
      const auto ci = bootstrap_mean_ci(values);
      const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
      all_output << mean << ',' << ci.first << ',' << ci.second;
    };
    for (const auto &[metric, families] : all_metrics) {
      for (const auto &[family, methods] : families) {
        const auto &baseline = methods.at("classical");
        for (const auto &[method, episodes] : methods) {
          std::vector<double> values, differences;
          for (const auto &[id, value] : episodes) {
            values.push_back(value);
            const auto paired = baseline.find(id);
            if (paired != baseline.end()) differences.push_back(value - paired->second);
          }
          all_output << csv_escape(metric) << ',' << csv_escape(family) << ','
                     << csv_escape(method) << ',' << groups.at(family).at(method).size()
                     << ',' << values.size() << ',';
          write_estimate(values);
          all_output << ',' << differences.size() << ',';
          write_estimate(differences);
          all_output << '\n';
        }
      }
    }
    if (!all_output) throw std::runtime_error("failed to write all-metric summary");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "aggregate_dispatch: " << error.what() << '\n';
    return 1;
  }
}
