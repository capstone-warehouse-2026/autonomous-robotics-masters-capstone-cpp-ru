#include "capstone_fleet_runtime/fleet_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <tuple>
#include <yaml-cpp/yaml.h>

namespace capstone_fleet_runtime {
namespace {
double age(std::chrono::steady_clock::time_point point) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - point).count();
}
std::string csv(const std::string &value) {
  std::string result = "\"";
  for (char c : value) { if (c == '"') result += '"'; result += c; }
  return result + '"';
}
void write_yaml(const std::filesystem::path &path, const YAML::Node &value) {
  std::ofstream stream(path);
  stream << value << '\n';
  if (!stream) throw std::runtime_error("cannot write " + path.string());
}
}  // namespace

FleetNode::FleetNode(const rclcpp::NodeOptions &options) : rclcpp::Node("fleet_coordinator", options) {
  read_parameters();
  scene_ = load_scene(manifest_path_);
  jobs_ = FleetJobQueue(scene_.mission.jobs);
  grid_ = scene_.grid;
  for (const auto &spec : scene_.robots) {
    Robot robot;
    robot.spec = spec;
    robot.cell = station_cell(scene_, spec.initial_station);
    robot.navigation = std::make_unique<RobotNavigator>(*this, spec.id, edge_timeout_);
    robots_.push_back(std::move(robot));
  }
  std::filesystem::create_directories(std::filesystem::path(output_directory_) / "plans");
  if (std::filesystem::exists(std::filesystem::path(output_directory_) / "events.csv") ||
      std::filesystem::exists(std::filesystem::path(output_directory_) / "fleet_episode.yaml")) {
    throw std::invalid_argument("fleet output already exists; choose a new directory");
  }
  events_.open(std::filesystem::path(output_directory_) / "events.csv");
  plans_.open(std::filesystem::path(output_directory_) / "plans.csv");
  events_ << "sim_time,event,job_id,robot_id,detail\n";
  plans_ << "plan_id,variant,robot_id,step,x,y\n";
  if (!events_ || !plans_) throw std::runtime_error("cannot create fleet evidence files");
  create_interfaces();
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { on_tick(); });
}

void FleetNode::read_parameters() {
  manifest_path_ = declare_parameter<std::string>("manifest_path", "");
  output_directory_ = declare_parameter<std::string>("output_directory", "");
  method_ = declare_parameter<std::string>("method", "cbs");
  plan_options_.weight = declare_parameter("weight", 1.5);
  plan_options_.clearance = declare_parameter("clearance", 0.85);
  plan_options_.max_time_steps = declare_parameter("max_time_steps", 64);
  const double planner_timeout = declare_parameter("planner_timeout", 5.0);
  footprint_radius_ = declare_parameter("planning_radius", 0.4);
  startup_timeout_ = declare_parameter("startup_timeout", 60.0);
  stop_timeout_ = declare_parameter("stop_timeout", 6.0);
  edge_timeout_ = declare_parameter("edge_timeout", 30.0);
  if (manifest_path_.empty() || output_directory_.empty() ||
      (method_ != "cbs" && method_ != "ecbs")) throw std::invalid_argument("invalid fleet parameters");
  for (double value : {planner_timeout, footprint_radius_, startup_timeout_, stop_timeout_, edge_timeout_}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("timeouts and radius must be positive");
  }
  if (!std::isfinite(plan_options_.weight) || plan_options_.weight < 1 ||
      !std::isfinite(plan_options_.clearance) || plan_options_.clearance < 0.85 ||
      footprint_radius_ < 0.4 || plan_options_.max_time_steps < 1) {
    throw std::invalid_argument("fleet geometry or search bounds violate the execution envelope");
  }
  plan_options_.wall_timeout = std::chrono::milliseconds(static_cast<int>(planner_timeout * 1000));
  plan_options_.method = method_ == "cbs" ? capstone_fleet::Method::CBS : capstone_fleet::Method::ECBS;
}

void FleetNode::create_interfaces() {
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "fleet/status", rclcpp::QoS(1).reliable().transient_local());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/" + robots_.front().spec.id + "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid &map) { on_map(map); });
  cancel_service_ = create_service<std_srvs::srv::Trigger>("fleet/cancel",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = !finished_;
        response->message = response->success ? "cancellation requested" : "fleet already terminal";
        if (response->success) begin_stop(State::Cancelled, "operator_cancel");
      });
}

void FleetNode::on_map(const nav_msgs::msg::OccupancyGrid &message) {
  if (map_ready_ || finished_ || state_ == State::Stopping) return;
  try {
    grid_ = grid_from_map(scene_.grid, message, footprint_radius_);
    for (const auto &station : scene_.mission.stations) {
      if (!grid_.free(station_cell(scene_, station.id))) {
        throw std::invalid_argument("station blocked by inflated map: " + station.id);
      }
    }
    map_ready_ = true;
    event("map_ready");
  } catch (const std::exception &error) {
    begin_stop(State::Fault, std::string("invalid_static_map:") + error.what());
  }
}

double FleetNode::elapsed() const { return std::max(0.0, now().seconds() - epoch_); }

void FleetNode::on_tick() {
  if (finished_) return;
  try {
    for (auto &robot : robots_) robot.navigation->update();
    if (state_ == State::Stopping) { poll_stopping(); publish_status(); return; }
    if (state_ == State::WaitingReady) { wait_ready(); publish_status(); return; }
    const double clock = now().seconds();
    if (clock < previous_clock_) { begin_stop(State::Fault, "unexpected_clock_reset"); return; }
    if (clock > previous_clock_) last_clock_progress_ = Clock::now();
    previous_clock_ = clock;
    if (age(last_clock_progress_) > 1.5) { begin_stop(State::Fault, "frozen_clock"); return; }
    for (const auto &robot : robots_) {
      if (!robot.navigation->inputs_fresh()) {
        begin_stop(State::Fault, "stale_navigation_input:" + robot.spec.id);
        return;
      }
    }
    if (elapsed() >= scene_.mission.horizon) { begin_stop(State::Fault, "episode_timeout"); return; }
    release_jobs();
    switch (state_) {
      case State::Idle: assign_and_plan(); break;
      case State::Planning: poll_planning(); break;
      case State::Executing: poll_round(); break;
      case State::Servicing: poll_services(); break;
      default: break;
    }
    publish_status();
  } catch (const std::exception &error) {
    begin_stop(State::Fault, std::string("runtime_error:") + error.what());
  }
}

void FleetNode::wait_ready() {
  bool ready = map_ready_;
  for (const auto &robot : robots_) {
    const auto [x, y] = grid_.world(robot.cell);
    ready = ready && robot.navigation->ready() && robot.navigation->settled_at(x, y);
  }
  if (!ready) ready_since_.reset();
  else if (!ready_since_) ready_since_ = Clock::now();
  // Первое совпадение request-topic ещё не гарантирует discovery обратного DDS reader.
  if (ready_since_ && age(*ready_since_) >= 1.0) {
    epoch_ = previous_clock_ = now().seconds();
    last_clock_progress_ = Clock::now();
    started_ = true;
    state_ = State::Idle;
    event("fleet_started", "", "", "method=" + method_);
  } else if (age(started_wall_) >= startup_timeout_) begin_stop(State::Fault, "startup_timeout");
}

void FleetNode::release_jobs() {
  std::vector<std::string> released;
  for (const auto &job : jobs_.jobs()) {
    if (job.state == capstone_mission::JobState::Unreleased && job.release_time <= elapsed()) {
      released.push_back(job.id);
    }
  }
  jobs_.release(elapsed());
  for (const auto &job : released) event("released", job);
}

void FleetNode::assign_jobs() {
  std::vector<std::size_t> idle;
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    if (robots_[i].phase == TaskPhase::Idle) idle.push_back(i);
  }
  std::vector<const capstone_mission::Job *> candidates;
  for (const auto &job : jobs_.jobs()) {
    if (job.state == capstone_mission::JobState::Released) candidates.push_back(&job);
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto *left, const auto *right) {
    return std::tie(left->due_time, left->id) < std::tie(right->due_time, right->id);
  });
  candidates.resize(std::min(candidates.size(), idle.size()));
  if (idle.empty() || candidates.empty()) return;
  std::vector<std::vector<int>> costs(idle.size(), std::vector<int>(candidates.size()));
  for (std::size_t i = 0; i < idle.size(); ++i) {
    for (std::size_t j = 0; j < candidates.size(); ++j) {
      costs[i][j] = capstone_fleet::shortest_distance(
          grid_, robots_[idle[i]].cell, station_cell(scene_, candidates[j]->pickup));
    }
  }
  const auto assignment = capstone_fleet::minimum_cost_assignment(costs);
  save_assignment(idle, candidates, costs, assignment);
  if (assignment.assignments.empty()) throw std::runtime_error("released jobs have no reachable pickup");
  std::set<std::size_t> assigned_robots, assigned_jobs;
  for (const auto &pair : assignment.assignments) {
    if (pair.robot >= idle.size() || pair.job >= candidates.size() ||
        !assigned_robots.insert(pair.robot).second || !assigned_jobs.insert(pair.job).second ||
        pair.cost < 0 || pair.cost != costs[pair.robot][pair.job]) {
      throw std::runtime_error("invalid assignment result");
    }
    auto &robot = robots_[idle[pair.robot]];
    const auto &job = *candidates[pair.job];
    jobs_.assign(job.id, robot.spec.id);
    robot.job = job.id;
    robot.phase = TaskPhase::ToPickup;
    event("assigned", job.id, robot.spec.id,
          "assignment=" + std::to_string(assignment_id_) + ";cost=" + std::to_string(pair.cost));
  }
}

void FleetNode::assign_and_plan() {
  if (jobs_.all_terminal()) {
    const bool delivered = std::all_of(jobs_.jobs().begin(), jobs_.jobs().end(), [](const auto &job) {
      return job.state == capstone_mission::JobState::Delivered;
    });
    if (delivered && all_stopped()) finish(State::Completed, "completed", true);
    else begin_stop(State::Fault, "unfinished_jobs");
    return;
  }
  assign_jobs();
  const bool active = std::any_of(robots_.begin(), robots_.end(), [](const auto &robot) {
    return robot.phase != TaskPhase::Idle;
  });
  if (active) start_planning();
}

void FleetNode::start_planning() {
  plan_starts_.clear();
  plan_goals_.clear();
  for (const auto &robot : robots_) {
    plan_starts_.push_back(robot.cell);
    auto goal = robot.cell;
    if (robot.phase != TaskPhase::Idle) {
      const auto &job = jobs_.find(robot.job);
      goal = station_cell(scene_, robot.phase == TaskPhase::ToPickup ? job.pickup : job.dropoff);
    }
    plan_goals_.push_back(goal);
  }
  ++plan_id_;
  state_ = State::Planning;
  event("planning_started", "", "", "plan=" + std::to_string(plan_id_));
  // Планирование ограничено своим deadline и не блокирует ROS callbacks.
  pending_plan_ = std::async(std::launch::async,
      [grid = grid_, starts = plan_starts_, goals = plan_goals_, options = plan_options_] {
        return capstone_fleet::plan(grid, starts, goals, options);
      });
}

void FleetNode::validate_joint_plan(const capstone_fleet::PlanResult &result) const {
  if (result.paths.size() != robots_.size() || result.paths.empty() || result.paths.front().empty()) {
    throw std::runtime_error("planner returned incomplete paths");
  }
  const auto length = result.paths.front().size();
  for (std::size_t i = 0; i < result.paths.size(); ++i) {
    const auto &path = result.paths[i];
    if (path.size() != length || path.front() != plan_starts_[i] || path.back() != plan_goals_[i]) {
      throw std::runtime_error("planner path endpoints or horizon mismatch");
    }
    for (std::size_t j = 1; j < path.size(); ++j) {
      if (!grid_.traversable(path[j - 1], path[j])) throw std::runtime_error("planner edge is blocked");
    }
  }
  if (result.final_conflicts != 0 ||
      !capstone_fleet::find_conflicts(grid_, result.paths, plan_options_.clearance).empty()) {
    throw std::runtime_error("planner returned a conflicting joint plan");
  }
}

void FleetNode::poll_planning() {
  if (pending_plan_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;
  const auto result = pending_plan_.get();
  save_plan(result);
  if (result.status != capstone_fleet::PlanStatus::Success) {
    begin_stop(State::Fault, "planning_failed:" + result.reason);
    return;
  }
  validate_joint_plan(result);
  joint_paths_ = result.paths;
  event("planning_succeeded", "", "", "plan=" + std::to_string(plan_id_) +
        ";initial_conflicts=" + std::to_string(result.initial_conflicts.size()));
  step_ = 1;
  start_round();
}

void FleetNode::start_round() {
  if (step_ >= joint_paths_.front().size()) { start_services(); return; }
  state_ = State::Executing;
  event("round_started", "", "", "plan=" + std::to_string(plan_id_) + ";step=" + std::to_string(step_));
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    auto &robot = robots_[i];
    const auto from = joint_paths_[i][step_ - 1], to = joint_paths_[i][step_];
    if (from != robot.cell) throw std::runtime_error("barrier cell differs from reserved edge");
    const auto [ax, ay] = grid_.world(from);
    const auto [bx, by] = grid_.world(to);
    if (from == to) robot.navigation->wait_at(bx, by);
    else robot.navigation->follow_edge(ax, ay, bx, by);
  }
}

void FleetNode::poll_round() {
  for (const auto &robot : robots_) {
    if (robot.navigation->state() == RobotNavigator::ActionState::Failed ||
        robot.navigation->state() == RobotNavigator::ActionState::Cancelled) {
      begin_stop(State::Fault, "navigation_failed:" + robot.spec.id + ":" + robot.navigation->reason());
      return;
    }
  }
  if (!std::all_of(robots_.begin(), robots_.end(), [](const auto &robot) {
        return robot.navigation->edge_complete();
      })) return;
  for (std::size_t i = 0; i < robots_.size(); ++i) robots_[i].cell = joint_paths_[i][step_];
  event("round_completed", "", "", "plan=" + std::to_string(plan_id_) + ";step=" + std::to_string(step_));
  ++step_;
  start_round();
}

void FleetNode::start_services() {
  state_ = State::Servicing;
  for (auto &robot : robots_) {
    const auto [x, y] = grid_.world(robot.cell);
    if (!robot.navigation->settled_at(x, y)) throw std::runtime_error("service started before physical stop");
    // После последнего ребра резервируется стоянка на конечной клетке, а не всё ребро.
    robot.navigation->wait_at(x, y);
    if (robot.phase == TaskPhase::Idle) continue;
    const auto &job = jobs_.find(robot.job);
    const bool pickup = robot.phase == TaskPhase::ToPickup;
    robot.phase = pickup ? TaskPhase::PickupService : TaskPhase::DropoffService;
    robot.service_until = elapsed() + (pickup ? job.pickup_service : job.dropoff_service);
    event(pickup ? "pickup_service" : "dropoff_service", robot.job, robot.spec.id);
  }
}

void FleetNode::poll_services() {
  bool pending = false;
  for (auto &robot : robots_) {
    if (robot.phase != TaskPhase::PickupService && robot.phase != TaskPhase::DropoffService) continue;
    const auto [x, y] = grid_.world(robot.cell);
    if (!robot.navigation->settled_at(x, y)) throw std::runtime_error("robot moved during station service");
    if (elapsed() < robot.service_until) { pending = true; continue; }
    if (robot.phase == TaskPhase::PickupService) {
      jobs_.mark_picked(robot.job, robot.spec.id, elapsed());
      event("picked", robot.job, robot.spec.id);
      robot.phase = TaskPhase::ToDropoff;
    } else {
      jobs_.mark_delivered(robot.job, robot.spec.id, elapsed());
      event("delivered", robot.job, robot.spec.id);
      robot.job.clear();
      robot.phase = TaskPhase::Idle;
    }
  }
  if (!pending) { state_ = State::Idle; assign_and_plan(); }
}

bool FleetNode::all_stopped() const {
  return std::all_of(robots_.begin(), robots_.end(), [](const auto &robot) {
    return robot.navigation->stop_complete();
  });
}

void FleetNode::begin_stop(State terminal, const std::string &reason) {
  if (finished_ || state_ == State::Stopping) return;
  requested_terminal_ = terminal;
  terminal_reason_ = reason;
  stopping_since_ = Clock::now();
  state_ = State::Stopping;
  event("stopping", "", "", reason);
  for (auto &robot : robots_) robot.navigation->cancel();
}

void FleetNode::poll_stopping() {
  if (all_stopped()) finish(requested_terminal_, terminal_reason_, true);
  else if (age(stopping_since_) > stop_timeout_) finish(State::Fault, terminal_reason_ + ":unconfirmed_stop", false);
}

void FleetNode::finish(State terminal, const std::string &reason, bool stopped) {
  state_ = terminal;
  terminal_reason_ = reason;
  event(terminal == State::Completed ? "fleet_completed" : "fleet_stopped", "", "", reason);
  YAML::Node output;
  output["episode_id"] = scene_.mission.episode_id;
  output["method"] = method_;
  output["terminal_reason"] = reason;
  output["terminal_state"] = status_name();
  output["physical_stop_confirmed_by_odometry"] = stopped;
  output["elapsed_sim_s"] = started_ ? elapsed() : 0.0;
  output["elapsed_wall_s"] = age(started_wall_);
  int delivered = 0;
  for (const auto &job : jobs_.jobs()) {
    YAML::Node item;
    item["id"] = job.id;
    item["robot_id"] = jobs_.owner(job.id);
    item["state"] = capstone_mission::to_string(job.state);
    item["picked_time"] = job.picked_time;
    item["delivered_time"] = job.delivered_time;
    output["jobs"].push_back(item);
    if (job.state == capstone_mission::JobState::Delivered) ++delivered;
  }
  output["delivered"] = delivered;
  output["total_jobs"] = scene_.mission.jobs.size();
  output["plans"] = plan_id_;
  output["assignments"] = assignment_id_;
  write_yaml(std::filesystem::path(output_directory_) / "fleet_episode.yaml", output);
  publish_status();
  exit_code_ = terminal == State::Completed ? 0 : 1;
  finished_ = true;
}

void FleetNode::request_shutdown() { begin_stop(State::Cancelled, "shutdown"); }

std::string FleetNode::status_name() const {
  switch (state_) {
    case State::WaitingReady: return "waiting";
    case State::Stopping: return "stopping";
    case State::Completed: return "completed";
    case State::Cancelled: return "cancelled";
    case State::Fault: return "fault";
    default: return "running";
  }
}

void FleetNode::publish_status() {
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "capstone_fleet/mission";
  status.message = status_name();
  status.level = state_ == State::Fault ? status.ERROR : status.OK;
  for (const auto &pair : std::vector<std::pair<std::string, std::string>>{
      {"method", method_}, {"episode_id", scene_.mission.episode_id},
      {"plan_id", std::to_string(plan_id_)}, {"step", std::to_string(step_)},
      {"reason", terminal_reason_}}) {
    diagnostic_msgs::msg::KeyValue value;
    value.key = pair.first; value.value = pair.second;
    status.values.push_back(value);
  }
  message.status.push_back(status);
  status_pub_->publish(message);
}

void FleetNode::event(const std::string &name, const std::string &job,
                      const std::string &robot, const std::string &detail) {
  events_ << std::setprecision(12) << now().seconds() << ',' << csv(name) << ','
          << csv(job) << ',' << csv(robot) << ',' << csv(detail) << '\n';
  events_.flush();
  if (!events_) throw std::runtime_error("cannot persist fleet event");
  RCLCPP_INFO(get_logger(), "event=%s robot=%s job=%s %s", name.c_str(), robot.c_str(), job.c_str(), detail.c_str());
}

void FleetNode::save_assignment(const std::vector<std::size_t> &idle,
                                const std::vector<const capstone_mission::Job *> &jobs,
                                const std::vector<std::vector<int>> &costs,
                                const capstone_fleet::AssignmentResult &assignment) {
  ++assignment_id_;
  YAML::Node output;
  output["assignment_id"] = assignment_id_;
  output["sim_time"] = now().seconds();
  output["policy"] = "EDF_batch_then_minimum_total_pickup_distance";
  for (auto i : idle) output["robot_ids"].push_back(robots_[i].spec.id);
  for (const auto *job : jobs) output["job_ids"].push_back(job->id);
  output["cost_matrix"] = costs;
  output["total_cost"] = assignment.total_cost;
  for (const auto &pair : assignment.assignments) {
    YAML::Node item;
    item["robot_id"] = robots_.at(idle.at(pair.robot)).spec.id;
    item["job_id"] = jobs.at(pair.job)->id;
    item["cost"] = pair.cost;
    output["chosen_pairs"].push_back(item);
  }
  write_yaml(std::filesystem::path(output_directory_) /
             ("assignment_" + std::to_string(assignment_id_) + ".yaml"), output);
}

void FleetNode::save_plan(const capstone_fleet::PlanResult &result) {
  for (const auto &entry : std::vector<std::pair<std::string, const capstone_fleet::Paths *>>{
      {"independent", &result.independent_paths}, {"resolved", &result.paths}}) {
    for (std::size_t robot = 0; robot < entry.second->size(); ++robot) {
      for (std::size_t step = 0; step < entry.second->at(robot).size(); ++step) {
        const auto [x, y] = grid_.world(entry.second->at(robot)[step]);
        plans_ << plan_id_ << ',' << entry.first << ',' << robots_.at(robot).spec.id << ','
               << step << ',' << std::setprecision(12) << x << ',' << y << '\n';
      }
    }
  }
  plans_.flush();
  if (!plans_) throw std::runtime_error("cannot persist joint paths");
  YAML::Node output;
  output["plan_id"] = plan_id_;
  output["method"] = method_;
  output["status"] = static_cast<int>(result.status);
  output["reason"] = result.reason;
  output["cost"] = result.cost;
  output["lower_bound"] = result.lower_bound;
  output["bound"] = result.suboptimality_bound;
  output["weight"] = plan_options_.weight;
  output["clearance"] = plan_options_.clearance;
  output["initial_conflicts"] = result.initial_conflicts.size();
  output["final_conflicts"] = result.final_conflicts;
  output["high_level_expansions"] = result.high_level_expansions;
  output["low_level_expansions"] = result.low_level_expansions;
  output["high_level_focal_slack_expansions"] = result.high_level_focal_slack_expansions;
  output["low_level_focal_slack_expansions"] = result.low_level_focal_slack_expansions;
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    YAML::Node robot;
    robot["robot_id"] = robots_[i].spec.id;
    const auto [sx, sy] = grid_.world(plan_starts_[i]);
    const auto [gx, gy] = grid_.world(plan_goals_[i]);
    robot["start"] = std::vector<double>{sx, sy};
    robot["goal"] = std::vector<double>{gx, gy};
    output["robots"].push_back(robot);
  }
  write_yaml(std::filesystem::path(output_directory_) / "plans" /
             ("plan_" + std::to_string(plan_id_) + ".yaml"), output);
}
}  // namespace capstone_fleet_runtime
