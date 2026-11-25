// Эталон Webots доступен только этому evaluator, а не алгоритмам навигации.
#include "capstone_system_test/route_evidence.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>

namespace capstone_system_test {
class SystemEvaluator : public rclcpp::Node {
 public:
  SystemEvaluator() : Node("system_evaluator") {
    read_parameters();
    prepare_output();
    create_interfaces();
    started_at_ = Steady::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { on_tick(); });
  }
  bool finished() const { return finished_; }
  int exit_code() const { return exit_code_; }
 private:
  using Steady = std::chrono::steady_clock;
  using Trigger = std_srvs::srv::Trigger;
  void read_parameters() {
    output_directory_ = declare_parameter<std::string>("output_directory", "/tmp/g5-system-audit");
    manifest_path_ = declare_parameter<std::string>("manifest_path", "");
    event_log_path_ = declare_parameter<std::string>("event_log", "");
    delivery_tolerance_ = declare_parameter("delivery_tolerance", 0.35);
    if (manifest_path_.empty() || event_log_path_.empty() ||
        !std::isfinite(delivery_tolerance_) || delivery_tolerance_ <= 0) {
      throw std::invalid_argument("manifest_path, event_log and positive delivery_tolerance required");
    }
    load_station_targets();
    expected_terminal_ = declare_parameter<std::string>("expected_terminal", "completed");
    const auto rectangle = declare_parameter<std::vector<double>>("obstacle", {-0.5, -2.0, 0.5, 2.0});
    const double radius = declare_parameter("robot_radius", 0.28);
    timeout_ = declare_parameter("wall_timeout", 240.0);
    cancel_distance_ = declare_parameter("cancel_after_distance", -1.0);
    require_detour_ = declare_parameter("require_detour", true);
    required_episodes_ = declare_parameter("required_episodes", 1);
    plan_topic_ = declare_parameter<std::string>("plan_topic", "plan");
    if (rectangle.size() != 4 || rectangle[0] >= rectangle[2] || rectangle[1] >= rectangle[3] ||
        !std::isfinite(radius) || radius < 0 || !std::isfinite(timeout_) || timeout_ <= 0 ||
        required_episodes_ < 1 || (expected_terminal_ != "completed" && expected_terminal_ != "cancelled")) {
      throw std::invalid_argument("invalid evaluator parameters");
    }
    obstacle_ = inflate({rectangle[0], rectangle[1], rectangle[2], rectangle[3]}, radius);
  }
  void load_station_targets() {
    const auto manifest = YAML::LoadFile(manifest_path_);
    std::map<std::string, Point> stations;
    for (const auto &station : manifest["stations"]) {
      const Point point{station["x"].as<double>(), station["y"].as<double>()};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !stations.emplace(station["id"].as<std::string>(), point).second) {
        throw std::invalid_argument("invalid station in physical test manifest");
      }
    }
    for (const auto &job : manifest["jobs"]) {
      const auto pickup = stations.at(job["pickup"].as<std::string>());
      const auto dropoff = stations.at(job["dropoff"].as<std::string>());
      if (!job_targets_.emplace(job["id"].as<std::string>(), std::make_pair(pickup, dropoff)).second) {
        throw std::invalid_argument("duplicate job in physical test manifest");
      }
    }
    if (job_targets_.empty()) throw std::invalid_argument("physical test requires deliveries");
  }
  void prepare_output() {
    std::filesystem::create_directories(output_directory_);
    const auto directory = std::filesystem::path(output_directory_);
    if (std::filesystem::exists(directory / "audit.yaml")) {
      throw std::invalid_argument("audit output already exists");
    }
    trajectory_.open(directory / "trajectory.csv");
    plans_.open(directory / "plans.csv");
    trajectory_ << "episode_id,source,sim_time,x,y,v,omega\n";
    plans_ << "episode_id,plan_id,index,sim_time,x,y\n";
    if (!trajectory_ || !plans_) throw std::runtime_error("cannot open evidence files");
  }
  void create_interfaces() {
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "evaluation/ground_truth", rclcpp::QoS(10).reliable(),
      [this](const nav_msgs::msg::Odometry &message) { on_ground_truth(message); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "state/odom", rclcpp::QoS(5).reliable(),
      [this](const nav_msgs::msg::Odometry &message) { on_odom(message); });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, rclcpp::QoS(10).reliable(),
      [this](const nav_msgs::msg::Path &message) { on_plan(message); });
    mission_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "mission/status", rclcpp::QoS(10).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray &message) { on_mission(message); });
    contact_sub_ = create_subscription<std_msgs::msg::Bool>(
      "evaluation/contact", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool &message) {
        if (active_episode_.empty()) return;
        ++contact_samples_;
        contact_received_ = Steady::now();
        if (message.data && !in_contact_) ++contact_events_;
        in_contact_ = message.data;
      });
    cancel_client_ = create_client<Trigger>("mission/cancel");
  }
  static double seconds(const builtin_interfaces::msg::Time &stamp) {
    return rclcpp::Time(stamp).seconds();
  }
  static bool finite(const nav_msgs::msg::Odometry &message) {
    return std::isfinite(message.pose.pose.position.x) && std::isfinite(message.pose.pose.position.y) &&
           std::isfinite(message.twist.twist.linear.x) && std::isfinite(message.twist.twist.angular.z);
  }
  double age(Steady::time_point point) const {
    return std::chrono::duration<double>(Steady::now() - point).count();
  }
  void write_sample(const std::string &source, const nav_msgs::msg::Odometry &message) {
    trajectory_ << std::setprecision(12) << active_episode_ << ',' << source << ','
                << seconds(message.header.stamp) << ',' << message.pose.pose.position.x << ','
                << message.pose.pose.position.y << ',' << message.twist.twist.linear.x << ','
                << message.twist.twist.angular.z << '\n';
  }
  void on_ground_truth(const nav_msgs::msg::Odometry &message) {
    if (!finite(message) || message.header.frame_id != "map") return;
    const double time = seconds(message.header.stamp);
    if (episodes_.size() > 0 && last_global_gt_time_ >= 0 &&
        time < last_global_gt_time_ - 0.1) ++resets_observed_;
    last_global_gt_time_ = time;
    if (active_episode_.empty()) return;
    const Point point{message.pose.pose.position.x, message.pose.pose.position.y};
    if (gt_samples_ && time <= gt_time_) return;
    double speed = 0;
    if (gt_samples_) {
      const double movement = std::hypot(point.x - gt_point_.x, point.y - gt_point_.y);
      speed = movement / (time - gt_time_);
      gt_distance_ += movement;
      if (intersects(obstacle_, gt_point_, point)) trajectory_intersection_ = true;
    }
    if (contains(obstacle_, point)) trajectory_intersection_ = true;
    gt_positions_.push_back({time, point});
    gt_point_ = point;
    gt_time_ = time;
    ++gt_samples_;
    gt_received_ = Steady::now();
    // Остановку подтверждают изменение истинной позы и измеренная скорость, а не одна команда ноль.
    if (speed < 0.03 && std::abs(message.twist.twist.angular.z) < 0.1) {
      if (gt_stopped_since_ < 0) gt_stopped_since_ = time;
    } else gt_stopped_since_ = -1;
    write_sample("ground_truth", message);
  }
  void on_odom(const nav_msgs::msg::Odometry &message) {
    if (!finite(message) || message.header.frame_id != "odom" || active_episode_.empty()) return;
    const double time = seconds(message.header.stamp);
    if (odom_samples_ && time <= odom_time_) return;
    odom_time_ = time;
    ++odom_samples_;
    odom_received_ = Steady::now();
    odom_stopped_ = std::abs(message.twist.twist.linear.x) < 0.05 &&
                    std::abs(message.twist.twist.angular.z) < 0.1;
    write_sample("wheel_odom", message);
  }
  void on_plan(const nav_msgs::msg::Path &message) {
    if (message.header.frame_id != "map" || active_episode_.empty()) return;
    std::vector<Point> points;
    for (const auto &pose : message.poses) {
      const Point point{pose.pose.position.x, pose.pose.position.y};
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) { finish(false, "nonfinite_plan"); return; }
      points.push_back(point);
      plans_ << std::setprecision(12) << active_episode_ << ',' << plan_count_ << ','
             << points.size() - 1 << ',' << seconds(message.header.stamp) << ','
             << point.x << ',' << point.y << '\n';
    }
    ++plan_count_;
    if (valid_detour(points, obstacle_)) detour_seen_ = true;
  }
  void on_mission(const diagnostic_msgs::msg::DiagnosticArray &message) {
    for (const auto &status : message.status) {
      if (status.name != "capstone_mission/mission") continue;
      std::string episode;
      for (const auto &value : status.values) if (value.key == "episode_id") episode = value.value;
      if (status.message == "running" && active_episode_.empty()) {
        begin_episode(episode, seconds(message.header.stamp));
      }
      if (episode != active_episode_ || active_episode_.empty()) continue;
      if (status.message == "completed" || status.message == "cancelled" || status.message == "fault") {
        if (terminal_.empty()) {
          terminal_ = status.message;
          terminal_time_ = seconds(message.header.stamp);
        }
      }
    }
  }
  void begin_episode(const std::string &episode, double time) {
    active_episode_ = episode;
    terminal_.clear();
    gt_positions_.clear();
    service_positions_ = YAML::Node(YAML::NodeType::Sequence);
    start_time_ = time;
    gt_samples_ = 0;
    odom_samples_ = 0;
    plan_count_ = 0;
    gt_distance_ = 0;
    gt_stopped_since_ = -1;
    trajectory_intersection_ = false;
    detour_seen_ = false;
    cancel_sent_ = false;
    cancel_accepted_ = false;
    contact_events_ = 0;
    contact_samples_ = 0;
    in_contact_ = false;
    RCLCPP_INFO(get_logger(), "Physical evidence episode %s", episode.c_str());
  }
  void request_cancel() {
    if (!cancel_client_->service_is_ready()) return;
    cancel_sent_ = true;
    cancel_time_ = gt_time_;
    cancel_client_->async_send_request(std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        cancel_accepted_ = future.get()->success;
        if (!cancel_accepted_) finish(false, "cancel_rejected");
      });
  }
  void on_tick() {
    if (finished_) return;
    if (age(started_at_) > timeout_) { finish(false, "wall_timeout"); return; }
    if (active_episode_.empty()) return;
    if (cancel_distance_ >= 0 && !cancel_sent_ && gt_distance_ >= cancel_distance_ && terminal_.empty()) {
      request_cancel();
    }
    if (terminal_.empty()) return;
    if (terminal_ != expected_terminal_) { finish(false, "unexpected_terminal_" + terminal_); return; }
    const bool stopped = odom_samples_ > 0 && odom_stopped_ && age(odom_received_) < 0.5 &&
                         gt_samples_ > 1 && age(gt_received_) < 0.5 && gt_stopped_since_ >= 0 &&
                         gt_time_ - std::max(terminal_time_, gt_stopped_since_) >= 0.3;
    if (!stopped) return;
    if (gt_distance_ < 0.3 || gt_samples_ < 10) { finish(false, "no_physical_motion_evidence"); return; }
    if (contact_samples_ == 0 || age(contact_received_) > 0.5) {
      finish(false, "missing_physical_contact_evidence"); return;
    }
    if (contact_events_ > 0) { finish(false, "physical_collision_contact"); return; }
    if (trajectory_intersection_) { finish(false, "physical_trajectory_intersects_shelf"); return; }
    if (require_detour_ && !detour_seen_) { finish(false, "no_obstacle_detour_plan"); return; }
    if (expected_terminal_ == "cancelled" && !cancel_accepted_) { finish(false, "cancel_not_accepted"); return; }
    const auto service_error = verify_service_positions();
    if (!service_error.empty()) { finish(false, service_error); return; }
    record_episode();
    if (static_cast<int>(episodes_.size()) >= required_episodes_) {
      if (required_episodes_ > 1 && resets_observed_ < required_episodes_ - 1) {
        finish(false, "missing_clock_reset_between_episodes");
      } else finish(true, "verified");
    }
    else {
      active_episode_.clear();
      RCLCPP_INFO(get_logger(), "Waiting for next reset and episode (%zu/%d)", episodes_.size(), required_episodes_);
    }
  }
  std::string verify_service_positions() {
    std::ifstream events(event_log_path_);
    std::string line;
    if (!std::getline(events, line) || line != "episode_id,job_id,sim_time,event,reason,target") {
      return "invalid_mission_event_log";
    }
    std::set<std::string> picked, delivered;
    while (std::getline(events, line)) {
      const auto fields = csv_row(line);
      if (fields.size() != 6) return "invalid_event_row";
      if (fields[0] != active_episode_ || (fields[3] != "picked" && fields[3] != "delivered")) continue;
      const auto target = job_targets_.find(fields[1]);
      if (target == job_targets_.end()) return "unknown_service_job";
      const bool pickup = fields[3] == "picked";
      auto &seen = pickup ? picked : delivered;
      if (!seen.insert(fields[1]).second || (!pickup && !picked.count(fields[1]))) {
        return "invalid_service_sequence";
      }
      const double time = start_time_ + std::stod(fields[2]);
      const auto actual = nearest_position(gt_positions_, time, 0.15);
      if (!actual) return "missing_ground_truth_at_service";
      const Point expected = pickup ? target->second.first : target->second.second;
      const double error = std::hypot(actual->x - expected.x, actual->y - expected.y);
      YAML::Node check;
      check["job_id"] = fields[1]; check["event"] = fields[3]; check["sim_time"] = time;
      check["actual_xy"] = std::vector<double>{actual->x, actual->y};
      check["target_xy"] = std::vector<double>{expected.x, expected.y};
      check["position_error_m"] = error;
      service_positions_.push_back(check);
      if (error > delivery_tolerance_) return "physical_service_outside_station_tolerance";
    }
    if (terminal_ == "completed" && delivered.size() != job_targets_.size()) {
      return "missing_physical_delivery_events";
    }
    return "";
  }
  void record_episode() {
    YAML::Node episode;
    episode["episode_id"] = active_episode_;
    episode["terminal"] = terminal_;
    episode["elapsed_sim_s"] = terminal_time_ - start_time_;
    episode["ground_truth_samples"] = gt_samples_;
    episode["wheel_odom_samples"] = odom_samples_;
    episode["ground_truth_distance_m"] = gt_distance_;
    episode["plan_count"] = plan_count_;
    episode["detour_plan_seen"] = detour_seen_;
    episode["physical_shelf_intersection"] = trajectory_intersection_;
    episode["physical_stop_confirmed"] = true;
    episode["physical_contact_events"] = contact_events_;
    episode["physical_contact_samples"] = contact_samples_;
    episode["service_position_checks"] = service_positions_;
    if (cancel_sent_) episode["cancel_to_stop_sim_s"] = gt_time_ - cancel_time_;
    episodes_.push_back(episode);
    const auto path = std::filesystem::path(output_directory_) /
        ("episode_" + std::to_string(episodes_.size()) + ".yaml");
    std::ofstream output(path);
    output << episode << '\n';
    trajectory_.flush();
    plans_.flush();
    if (!output || !trajectory_ || !plans_) throw std::runtime_error("cannot persist episode evidence");
  }
  void finish(bool success, const std::string &reason) {
    if (finished_) return;
    YAML::Node result;
    result["success"] = success;
    result["reason"] = reason;
    result["scope"] = "physical_Webots_Nav2_G5_integration";
    result["expected_terminal"] = expected_terminal_;
    result["required_episodes"] = required_episodes_;
    result["completed_audits"] = episodes_.size();
    result["time_resets_observed"] = resets_observed_;
    result["active_episode"] = active_episode_;
    result["current_ground_truth_samples"] = gt_samples_;
    result["current_plan_count"] = plan_count_;
    result["current_contact_events"] = contact_events_;
    result["current_service_position_checks"] = service_positions_;
    result["delivery_tolerance_m"] = delivery_tolerance_;
    result["episodes"] = episodes_;
    result["obstacle_inflated"] = std::vector<double>{obstacle_.xmin, obstacle_.ymin, obstacle_.xmax, obstacle_.ymax};
    std::ofstream output(std::filesystem::path(output_directory_) / "audit.yaml");
    output << result << '\n';
    trajectory_.flush(); plans_.flush();
    if (!output || !trajectory_ || !plans_) throw std::runtime_error("cannot persist physical evidence");
    finished_ = true;
    exit_code_ = success ? 0 : 1;
    RCLCPP_INFO(get_logger(), "Physical audit: %s", reason.c_str());
  }
  std::string output_directory_, expected_terminal_, plan_topic_, active_episode_, terminal_;
  std::string manifest_path_, event_log_path_;
  double delivery_tolerance_{0.35};
  std::map<std::string, std::pair<Point, Point>> job_targets_;
  std::vector<PositionSample> gt_positions_;
  YAML::Node service_positions_{YAML::NodeType::Sequence};
  Rectangle obstacle_{};
  double timeout_{240}, cancel_distance_{-1}, start_time_{0}, terminal_time_{0}, cancel_time_{0};
  double gt_time_{0}, odom_time_{0}, gt_stopped_since_{-1}, gt_distance_{0}, last_global_gt_time_{-1};
  int required_episodes_{1}, exit_code_{0}, resets_observed_{0};
  std::size_t gt_samples_{0}, odom_samples_{0}, plan_count_{0};
  bool finished_{false}, require_detour_{true}, detour_seen_{false}, trajectory_intersection_{false};
  bool odom_stopped_{false}, cancel_sent_{false}, cancel_accepted_{false}, in_contact_{false};
  int contact_events_{0}, contact_samples_{0};
  Point gt_point_{};
  Steady::time_point started_at_, gt_received_, odom_received_, contact_received_;
  std::ofstream trajectory_, plans_;
  YAML::Node episodes_{YAML::NodeType::Sequence};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_, odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace capstone_system_test

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<capstone_system_test::SystemEvaluator>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->finished()) executor.spin_once(std::chrono::milliseconds(100));
    const int code = node->finished() ? node->exit_code() : 1;
    rclcpp::shutdown();
    return code;
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("system_evaluator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
