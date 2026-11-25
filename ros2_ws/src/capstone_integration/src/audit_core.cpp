#include "capstone_integration/audit_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace capstone_integration {

namespace {

std::string escape_json(const std::string &value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        break;
      case '\t':
        out += " ";
        break;
      default:
        out.push_back(c);
    }
  }
  return out;
}

std::string number(double value, int decimals = 1) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

std::string base_name(const std::string &node) {
  const auto position = node.find_last_of('/');
  return position == std::string::npos ? node : node.substr(position + 1);
}

bool looks_like_evaluator(const std::string &node) {
  const std::string name = base_name(node);
  return name == "benchmark_runner" || name.find("eval") != std::string::npos;
}

}  // namespace

Row evaluate_topic(const TopicSample &sample) {
  const Contract &contract = sample.contract;
  if (!sample.exists) {
    return Row{contract.name, contract.required ? Verdict::Fail : Verdict::Info,
               contract.required ? "topic not found" : "not present (optional)"};
  }
  if (!contract.type.empty() && sample.actual_type != contract.type) {
    return Row{contract.name, Verdict::Fail,
               "type " + sample.actual_type + ", expected " + contract.type};
  }
  if (sample.publishers < contract.min_publishers) {
    return Row{contract.name, Verdict::Fail,
               std::to_string(sample.publishers) + " publisher(s), expected >= " +
                   std::to_string(contract.min_publishers)};
  }
  if (sample.publishers > contract.max_publishers) {
    return Row{contract.name, Verdict::Fail,
               std::to_string(sample.publishers) + " publisher(s), expected <= " +
                   std::to_string(contract.max_publishers) +
                   " (duplicate publisher is an integration defect)"};
  }
  if (contract.presence_only) {
    return Row{contract.name, Verdict::Ok, "present, " + std::to_string(sample.messages) +
                                              " message(s) in the window"};
  }
  if (sample.messages == 0) {
    return Row{contract.name, Verdict::Fail, "no messages in the window"};
  }
  if (!contract.frame.empty() && sample.frame != contract.frame) {
    return Row{contract.name, Verdict::Fail,
               "frame '" + sample.frame + "', expected '" + contract.frame + "'"};
  }
  const double low = contract.min_hz * 0.8;
  const double high = contract.max_hz * 1.2;
  if (sample.measured_hz < low || sample.measured_hz > high) {
    return Row{contract.name, Verdict::Warn,
               number(sample.measured_hz) + " Hz, expected about " + number(contract.min_hz) +
                   " Hz"};
  }
  if (contract.frame.empty()) {
    return Row{contract.name, Verdict::Ok, number(sample.measured_hz) + " Hz"};
  }
  return Row{contract.name, Verdict::Ok,
             number(sample.measured_hz) + " Hz, frame '" + sample.frame + "'"};
}

Row evaluate_clock(int messages, double first_s, double last_s, bool monotonic, int publishers) {
  if (publishers != 1) {
    return Row{"/clock", Verdict::Fail,
               std::to_string(publishers) + " publisher(s), expected exactly 1"};
  }
  if (messages == 0) {
    return Row{"/clock", Verdict::Fail, "no messages: the simulator is not stepping"};
  }
  if (!monotonic) {
    return Row{"/clock", Verdict::Fail, "clock went backwards without a reset"};
  }
  if (!(last_s > first_s)) {
    return Row{"/clock", Verdict::Fail, "clock frozen: nothing advances, the mission cannot time out"};
  }
  return Row{"/clock", Verdict::Ok, "advancing"};
}

Row evaluate_action(const std::string &name, bool present) {
  return present ? Row{name, Verdict::Ok, "action server present"}
                 : Row{name, Verdict::Fail, "action server not found"};
}

Row evaluate_tf(bool map_to_base, bool map_to_odom, bool odom_to_base) {
  if (map_to_base) {
    return Row{"TF map->base_link", Verdict::Ok, "chain resolves"};
  }
  std::string missing;
  if (!map_to_odom) {
    missing += "map->odom ";
  }
  if (!odom_to_base) {
    missing += "odom->base_link ";
  }
  return Row{"TF map->base_link", Verdict::Fail, "missing link(s): " + missing};
}

Row evaluate_placeholders(const std::vector<std::string> &nodes, bool measured_run) {
  std::string active;
  for (const auto &node : nodes) {
    const std::string name = base_name(node);
    for (const char *placeholder : {"world_stub", "path_stub", "drive_stub", "mission_stub"}) {
      if (name == placeholder) {
        active += active.empty() ? name : ", " + name;
      }
    }
  }
  if (active.empty()) {
    return Row{"placeholders", Verdict::Ok, "no placeholder nodes running"};
  }
  if (measured_run) {
    return Row{"placeholders", Verdict::Fail,
               "placeholder nodes in a measured run: " + active +
                   " (results are not valid system results)"};
  }
  return Row{"placeholders", Verdict::Warn, "placeholder nodes active: " + active};
}

Row evaluate_ground_truth(const std::vector<std::string> &subscriber_nodes) {
  std::string leaked;
  for (const auto &node : subscriber_nodes) {
    if (!looks_like_evaluator(node)) {
      leaked += leaked.empty() ? node : ", " + node;
    }
  }
  if (leaked.empty()) {
    return Row{"ground_truth_leak", Verdict::Ok, "no runtime subscriber"};
  }
  return Row{"ground_truth_leak", Verdict::Fail,
             "runtime nodes subscribe to /evaluation/ground_truth: " + leaked};
}

std::string verdict_name(Verdict verdict) {
  switch (verdict) {
    case Verdict::Ok:
      return "OK";
    case Verdict::Warn:
      return "WARN";
    case Verdict::Fail:
      return "FAIL";
    case Verdict::Info:
      return "INFO";
  }
  return "?";
}

std::string format_table(const std::vector<Row> &rows) {
  std::ostringstream out;
  out << "TOPIC / CHECK                    STATUS  DETAIL\n";
  for (const auto &row : rows) {
    std::string name = row.name;
    if (name.size() > 31) {
      name = name.substr(0, 28) + "...";
    }
    out << name;
    out << std::string(33 - name.size(), ' ');
    const std::string status = verdict_name(row.verdict);
    out << status << std::string(8 - status.size(), ' ');
    out << row.detail << '\n';
  }
  return out.str();
}

std::string to_json(const std::vector<Row> &rows, double window_s, bool measured_run) {
  std::ostringstream out;
  out << "{\n";
  out << "  \"window_s\": " << number(window_s, 2) << ",\n";
  out << "  \"measured_run\": " << (measured_run ? "true" : "false") << ",\n";
  int failures = 0;
  int warnings = 0;
  for (const auto &row : rows) {
    failures += row.verdict == Verdict::Fail ? 1 : 0;
    warnings += row.verdict == Verdict::Warn ? 1 : 0;
  }
  out << "  \"failures\": " << failures << ",\n";
  out << "  \"warnings\": " << warnings << ",\n";
  out << "  \"checks\": [\n";
  for (std::size_t i = 0; i < rows.size(); ++i) {
    out << "    {\"name\": \"" << escape_json(rows[i].name) << "\", \"verdict\": \""
        << verdict_name(rows[i].verdict) << "\", \"detail\": \"" << escape_json(rows[i].detail)
        << "\"}" << (i + 1 < rows.size() ? "," : "") << '\n';
  }
  out << "  ]\n}\n";
  return out.str();
}

int exit_code(const std::vector<Row> &rows, const std::string &fail_on) {
  if (fail_on == "never") {
    return 0;
  }
  for (const auto &row : rows) {
    if (row.verdict == Verdict::Fail) {
      return 1;
    }
    if (fail_on == "warn" && row.verdict == Verdict::Warn) {
      return 1;
    }
  }
  return 0;
}

}  // namespace capstone_integration
