// Запуск замороженных табличных эпизодов; метрики вычисляются только C++.
#include "capstone_benchmark/episode.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using namespace capstone_benchmark;
using namespace capstone_mission;

namespace {
std::string read_file(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot read " + path.string());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
std::string sha256(const fs::path &path) {
  const auto contents = read_file(path);
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(contents.data()), contents.size(), digest);
  std::ostringstream value;
  for (unsigned char byte : digest) value << std::hex << std::setw(2) << std::setfill('0') << int(byte);
  return value.str();
}
struct ModelEntry { std::string name, path; bool congestion{true}; };
}

int main(int argc, char **argv) {
  try {
    std::string scenarios, output, commit = "unrecorded";
    std::vector<ModelEntry> entries{{"classical", "", true}};
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--help") {
        std::cout << "dispatch_benchmark --scenarios DIR --output DIR [--model NAME=FILE] "
                     "[--ablation NAME=FILE] [--commit SHA]\n";
        return 0;
      }
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + option);
      const std::string value = argv[++i];
      if (option == "--scenarios") scenarios = value;
      else if (option == "--output") output = value;
      else if (option == "--commit") commit = value;
      else if (option == "--model" || option == "--ablation") {
        const auto equals = value.find('=');
        if (equals == std::string::npos) throw std::invalid_argument("model requires NAME=FILE");
        entries.push_back({value.substr(0, equals), value.substr(equals + 1), option == "--model"});
      } else throw std::invalid_argument("unknown option " + option);
    }
    if (scenarios.empty() || output.empty()) throw std::invalid_argument("scenarios/output required");
    if (fs::exists(fs::path(output) / "metrics.csv")) {
      throw std::invalid_argument("output already contains metrics.csv; choose a new run directory");
    }
    fs::create_directories(output);
    std::vector<fs::path> paths;
    for (const auto &entry : fs::directory_iterator(scenarios)) {
      if (entry.is_regular_file() && (entry.path().extension() == ".json" ||
                                      entry.path().extension() == ".yaml")) paths.push_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    if (paths.empty()) throw std::invalid_argument("no scenario manifests found");
    std::ofstream metrics(fs::path(output) / "metrics.csv");
    std::ofstream events(fs::path(output) / "events.csv");
    metrics << metrics_header() << '\n';
    events << "episode_id,method,job_id,sim_time,event,reason,target\n";
    YAML::Node manifest;
    manifest["schema_version"] = 1;
    manifest["evaluation"] = "isolated_dispatch_tables";
    manifest["code_commit"] = commit;
    manifest["analysis_seed"] = 2026;
    manifest["compiler"] = __VERSION__;
    manifest["scope"] = "No Webots physics, perception, localization, or collision metrics";
    std::set<std::string> names;
    for (const auto &entry : entries) {
      if (entry.name.empty() || !names.insert(entry.name).second) {
        throw std::invalid_argument("duplicate or empty method name");
      }
      std::unique_ptr<DqnModel> model;
      std::string model_error;
      if (!entry.path.empty()) {
        try { model = std::make_unique<DqnModel>(entry.path); }
        catch (const std::exception &error) { model_error = error.what(); }
      }
      auto model_info = manifest["methods"][entry.name];
      model_info["congestion_feature"] = entry.congestion;
      model_info["model_error"] = model_error;
      if (!entry.path.empty() && fs::exists(entry.path)) model_info["model_sha256"] = sha256(entry.path);
      std::set<std::string> ids;
      for (const auto &path : paths) {
        const Scenario scenario = load_scenario(path.string());
        if (!ids.insert(scenario.episode_id).second) throw std::invalid_argument("duplicate episode ID");
        const auto result = run_episode(scenario, entry.name, model.get(), entry.congestion);
        metrics << metrics_row(result.metrics) << '\n';
        for (const auto &event : result.events) {
          events << csv_escape(event.episode_id) << ',' << csv_escape(entry.name) << ','
                 << csv_escape(event.job_id) << ',' << std::setprecision(12) << event.sim_time << ','
                 << csv_escape(event.event) << ',' << csv_escape(event.reason) << ','
                 << csv_escape(event.target) << '\n';
        }
        manifest["scenarios"][scenario.episode_id]["sha256"] = sha256(path);
        manifest["scenarios"][scenario.episode_id]["seed"] = scenario.seed;
        manifest["scenarios"][scenario.episode_id]["split"] = scenario.split;
        manifest["scenarios"][scenario.episode_id]["family"] = scenario.family;
        manifest["scenarios"][scenario.episode_id]["layout_id"] = scenario.layout_id;
        // Копия сценария позволяет повторить запуск после изменения исходного каталога.
        fs::create_directories(fs::path(output) / "scenarios");
        fs::copy_file(path, fs::path(output) / "scenarios" / path.filename(),
                      fs::copy_options::overwrite_existing);
      }
      std::cout << entry.name << ": " << paths.size() << " episodes\n";
    }
    std::ofstream manifest_file(fs::path(output) / "manifest.yaml");
    manifest_file << manifest;
    if (!metrics || !events || !manifest_file) throw std::runtime_error("failed to write results");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "dispatch_benchmark: " << error.what() << '\n';
    return 1;
  }
}
