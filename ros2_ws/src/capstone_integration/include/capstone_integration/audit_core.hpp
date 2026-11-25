#pragma once

#include <string>
#include <vector>

#include "capstone_integration/contracts.hpp"

namespace capstone_integration {

enum class Verdict { Info, Ok, Warn, Fail };

struct TopicSample {
  Contract contract;
  bool exists{false};
  std::string actual_type;
  int publishers{0};
  int messages{0};
  double measured_hz{0.0};
  std::string frame;
};

struct Row {
  std::string name;
  Verdict verdict{Verdict::Info};
  std::string detail;
};

Row evaluate_topic(const TopicSample &sample);
Row evaluate_clock(int messages, double first_s, double last_s, bool monotonic, int publishers);
Row evaluate_action(const std::string &name, bool present);
Row evaluate_tf(bool map_to_base, bool map_to_odom, bool odom_to_base);
Row evaluate_placeholders(const std::vector<std::string> &nodes, bool measured_run);
Row evaluate_ground_truth(const std::vector<std::string> &subscriber_nodes);

std::string verdict_name(Verdict verdict);
std::string format_table(const std::vector<Row> &rows);
std::string to_json(const std::vector<Row> &rows, double window_s, bool measured_run);
int exit_code(const std::vector<Row> &rows, const std::string &fail_on);

}  // namespace capstone_integration
