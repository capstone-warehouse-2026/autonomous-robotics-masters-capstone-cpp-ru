#include "capstone_mission/mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace capstone_mission {
namespace {
// Все листья используют один неблокирующий механизм; состояние миссии хранится в узле.
class MissionAction : public BT::StatefulActionNode {
 public:
  using Tick = std::function<BT::NodeStatus()>;
  MissionAction(const std::string &name, const BT::NodeConfig &config, Tick tick,
                std::function<void()> halt)
      : BT::StatefulActionNode(name, config), tick_(std::move(tick)), halt_(std::move(halt)) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return tick_(); }
  BT::NodeStatus onRunning() override { return tick_(); }
  void onHalted() override { halt_(); }
 private:
  Tick tick_;
  std::function<void()> halt_;
};

std::string csv(const std::string &value) {
  std::string result = "\"";
  for (char character : value) {
    if (character == '"') result += '"';
    result += character;
  }
  return result + '"';
}

double elapsed(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

MissionNode::MissionNode(const rclcpp::NodeOptions &options) : rclcpp::Node("mission", options) {
  read_parameters();
  load_configuration();
  create_ros_interfaces();
  navigation_ = std::make_unique<NavigationClient>(*this, navigation_limits_,
      [this](const std::string &event, const std::string &reason) {
        record_event(event, reason, navigation_target_);
      });
  if (configured_) create_behavior_tree();
  // Конфигурация фиксируется на запуск: смена метода не должна менять текущий эксперимент.
  parameter_guard_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "mission parameters are immutable; restart between experiments";
        return result;
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { on_timer(); });
  if (configured_ && autostart_) start_episode();
}

void MissionNode::read_parameters() {
  method_ = declare_parameter<std::string>("method", "classical");
  if (method_ != "classical" && method_ != "learned") {
    throw std::invalid_argument("method must be 'classical' or 'learned'");
  }
  manifest_path_ = declare_parameter<std::string>("manifest_path", "");
  model_path_ = declare_parameter<std::string>("model_path", "");
  tree_path_ = declare_parameter<std::string>("bt_xml", "");
  event_log_path_ = declare_parameter<std::string>("event_log", "mission_events.csv");
  autostart_ = declare_parameter<bool>("autostart", true);
  require_safety_ = declare_parameter<bool>("require_safety", true);
  congestion_feature_ = declare_parameter<bool>("congestion_feature", true);
  clock_timeout_ = declare_parameter<double>("clock_timeout", 2.0);
  readiness_timeout_ = declare_parameter<double>("readiness_timeout", 15.0);
  navigation_limits_.planning_timeout = declare_parameter<double>("planning_timeout", 0.2);
  navigation_limits_.navigation_timeout = declare_parameter<double>("navigation_timeout", 120.0);
  navigation_limits_.stop_timeout = declare_parameter<double>("stop_timeout", 2.0);
  navigation_limits_.stop_confirmation_time = declare_parameter<double>("stop_confirmation_time", 0.2);
  navigation_limits_.input_timeout = declare_parameter<double>("input_timeout", 0.3);
  if (!std::isfinite(clock_timeout_) || clock_timeout_ <= 0.0 ||
      !std::isfinite(readiness_timeout_) || readiness_timeout_ <= 0.0) {
    throw std::invalid_argument("clock/readiness timeouts must be finite and positive");
  }
}

void MissionNode::load_configuration() {
  if (manifest_path_.empty()) return;
  scenario_ = load_scenario(manifest_path_);
  queue_ = JobQueue(scenario_.jobs);
  configured_ = true;
  if (tree_path_.empty()) {
    tree_path_ = ament_index_cpp::get_package_share_directory("capstone_mission") +
                 "/behavior_trees/delivery.xml";
  }
  if (method_ == "learned") {
    if (model_path_.empty()) {
      model_path_ = ament_index_cpp::get_package_share_directory("capstone_mission") +
                    "/models/dispatch_dqn.json";
    }
    try {
      model_ = std::make_unique<DqnModel>(model_path_);
    } catch (const std::exception &error) {
      model_error_ = error.what();
      RCLCPP_WARN(get_logger(), "DQN unavailable: %s; using classical fallback", error.what());
    }
  }
  if (!event_log_path_.empty()) {
    const auto directory = std::filesystem::path(event_log_path_).parent_path();
    if (!directory.empty()) std::filesystem::create_directories(directory);
    const bool write_header = !std::filesystem::exists(event_log_path_) ||
                             std::filesystem::file_size(event_log_path_) == 0;
    events_.open(event_log_path_, std::ios::app);
    if (!events_) throw std::runtime_error("cannot open event log: " + event_log_path_);
    if (write_header) events_ << "episode_id,job_id,sim_time,event,reason,target\n";
  }
}

void MissionNode::create_ros_interfaces() {
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "mission/status", rclcpp::QoS(10).reliable());
  diagnostic_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray &message) { on_diagnostics(message); });
  localization_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", rclcpp::QoS(5).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
        on_localization(message);
      });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "sensors/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan &message) {
        if (message.header.frame_id.empty() || message.ranges.empty() ||
            !std::isfinite(message.angle_increment) || message.angle_increment <= 0.0) return;
        receive_sensor_stamp(message.header.stamp, scan_stamp_, scan_received_, pending_scan_);
      });
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "perception/obstacles", rclcpp::QoS(1).reliable(),
      [this](const nav_msgs::msg::OccupancyGrid &message) {
        if (message.header.frame_id != "odom" || message.info.width == 0 ||
            message.info.height == 0 || !std::isfinite(message.info.resolution) ||
            message.info.resolution <= 0.0 ||
            message.data.size() != static_cast<std::size_t>(message.info.width) * message.info.height) return;
        receive_sensor_stamp(message.header.stamp, grid_stamp_, grid_received_, pending_grid_);
      });
  start_service_ = create_service<Trigger>("mission/start",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        response->success = configured_ && state_ == State::Idle;
        response->message = response->success ? "mission started" : "mission not configured or not idle";
        if (response->success) start_episode();
      });
  cancel_service_ = create_service<Trigger>("mission/cancel",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        response->success = state_ == State::Running;
        response->message = response->success ? "cancel requested; await mission/status" : "no active mission";
        if (response->success) begin_cancel("operator_cancel");
      });
  reset_service_ = create_service<Trigger>("mission/reset",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        const bool resolved_stop = navigation_->safe_to_retry() ||
            (navigation_->state() == NavigationClient::State::Idle && navigation_->stopped());
        response->success = configured_ && state_ != State::Cancelling &&
                            (state_ != State::Fault || resolved_stop);
        response->message = response->success ? "reset requested; await mission/status" : "unsafe or unconfigured reset";
        if (!response->success) return;
        if (state_ == State::Running) begin_cancel("episode_reset", true);
        else reset_episode(autostart_);
      });
}

void MissionNode::create_behavior_tree() {
  auto add = [this](const std::string &name, MissionAction::Tick tick, bool navigation = false) {
    factory_.registerBuilder<MissionAction>(name,
        [this, tick, navigation](const std::string &instance, const BT::NodeConfig &config) {
          return std::make_unique<MissionAction>(instance, config, tick, [this, navigation] {
            if (navigation && navigation_->state() != NavigationClient::State::Idle &&
                navigation_->state() != NavigationClient::State::Succeeded &&
                navigation_->state() != NavigationClient::State::Failed) {
              navigation_->cancel("tree_halted");
            }
          });
        });
  };
  add("WaitReady", [this] { return wait_ready(); });
  add("SelectJob", [this] { return select_job(); });
  add("NavigatePickup", [this] { return navigate(true); }, true);
  add("PickupService", [this] { return service(true); });
  add("NavigateDropoff", [this] { return navigate(false); }, true);
  add("DropoffService", [this] { return service(false); });
  add("CompleteJob", [this] { return complete_job(); });
  tree_ = factory_.createTreeFromFile(tree_path_);
}

void MissionNode::start_episode() {
  if (episode_id_.empty() || state_ != State::Idle) {
    reset_episode(true);
    return;
  }
  epoch_ = now().seconds();
  last_clock_ = epoch_;
  clock_seen_ = !get_parameter("use_sim_time").as_bool() || epoch_ > 0.0;
  last_clock_progress_ = episode_started_ = Clock::now();
  state_ = State::Running;
  record_event("episode_started");
}

void MissionNode::reset_episode(bool activate) {
  tree_.haltTree();
  navigation_->clear();
  queue_.reset();
  active_job_.clear();
  navigation_target_.clear();
  episode_id_ = scenario_.episode_id + "-" + std::to_string(episode_sequence_++);
  epoch_ = now().seconds();
  last_clock_ = epoch_;
  clock_seen_ = !get_parameter("use_sim_time").as_bool() || epoch_ > 0.0;
  last_clock_progress_ = episode_started_ = Clock::now();
  attempts_ = navigation_failures_ = fallback_count_ = 0;
  wait_until_ = retry_at_ = 0.0;
  service_started_ = reset_after_cancel_ = false;
  state_ = activate ? State::Running : State::Idle;
  record_event(activate ? "episode_started" : "episode_ready");
}

double MissionNode::episode_time() const { return std::max(0.0, now().seconds() - epoch_); }

void MissionNode::on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray &message) {
  for (const auto &status : message.status) {
    if (status.name == "capstone_safety/safety_monitor") {
      safety_received_ = Clock::now();
      safety_reason_ = status.message;
    }
  }
}

void MissionNode::receive_sensor_stamp(const builtin_interfaces::msg::Time &incoming,
                                       rclcpp::Time &accepted, Clock::time_point &received,
                                       std::optional<PendingStamp> &pending) {
  if (incoming.sec < 0 || incoming.nanosec >= 1000000000) return;
  process_pending_inputs();
  const rclcpp::Time stamp(incoming);
  const double age = (now() - stamp).seconds();
  if (age < 0.0) {
    if (age >= -navigation_limits_.input_timeout && !pending) {
      pending = PendingStamp{stamp, Clock::now()};
    }
    return;
  }
  if (age <= navigation_limits_.input_timeout && stamp >= accepted) {
    accepted = stamp;
    received = Clock::now();
  }
}

void MissionNode::on_localization(
    const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
  if (message.header.frame_id != "map" || !std::isfinite(message.pose.pose.position.x) ||
      !std::isfinite(message.pose.pose.position.y) || message.header.stamp.sec < 0 ||
      message.header.stamp.nanosec >= 1000000000) return;
  process_pending_inputs();
  const rclcpp::Time stamp(message.header.stamp);
  const double age = (now() - stamp).seconds();
  if (age < 0.0) {
    if (age >= -navigation_limits_.input_timeout && !pending_localization_) {
      pending_localization_ = PendingLocalization{message, Clock::now()};
    }
    return;
  }
  if (age <= navigation_limits_.input_timeout &&
      stamp >= rclcpp::Time(localization_.header.stamp)) {
    localization_ = message;
    localization_received_ = Clock::now();
  }
}

void MissionNode::process_pending_inputs() {
  const double current = now().seconds();
  if (previous_input_clock_ && current < *previous_input_clock_) {
    pending_scan_.reset();
    pending_grid_.reset();
    pending_localization_.reset();
    scan_stamp_ = grid_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    localization_ = geometry_msgs::msg::PoseWithCovarianceStamped{};
    scan_received_ = grid_received_ = localization_received_ = Clock::time_point{};
  }
  previous_input_clock_ = current;
  // Один ожидающий stamp на вход. Его исходное steady-время не меняется при ожидании.
  auto promote_stamp = [this, current](std::optional<PendingStamp> &pending,
                                      rclcpp::Time &accepted, Clock::time_point &received) {
    if (!pending) return;
    const double age = current - pending->stamp.seconds();
    if (elapsed(pending->received_at) > navigation_limits_.input_timeout ||
        age > navigation_limits_.input_timeout || age < -navigation_limits_.input_timeout) {
      pending.reset();
    } else if (age >= 0.0) {
      if (pending->stamp >= accepted) {
        accepted = pending->stamp;
        received = pending->received_at;
      }
      pending.reset();
    }
  };
  promote_stamp(pending_scan_, scan_stamp_, scan_received_);
  promote_stamp(pending_grid_, grid_stamp_, grid_received_);
  if (!pending_localization_) return;
  const auto &pending = *pending_localization_;
  const rclcpp::Time stamp(pending.message.header.stamp);
  const double age = current - stamp.seconds();
  if (elapsed(pending.received_at) > navigation_limits_.input_timeout ||
      age > navigation_limits_.input_timeout || age < -navigation_limits_.input_timeout) {
    pending_localization_.reset();
  } else if (age >= 0.0) {
    if (stamp >= rclcpp::Time(localization_.header.stamp)) {
      localization_ = pending.message;
      localization_received_ = pending.received_at;
    }
    pending_localization_.reset();
  }
}

bool MissionNode::fresh_localization() const {
  const double age = (now() - rclcpp::Time(localization_.header.stamp)).seconds();
  return localization_.header.frame_id == "map" && age >= -0.01 &&
         age <= navigation_limits_.input_timeout &&
         elapsed(localization_received_) <= navigation_limits_.input_timeout;
}

bool MissionNode::safety_inputs_fresh() const {
  if (!require_safety_) return true;
  const double scan_age = (now() - scan_stamp_).seconds();
  const double grid_age = (now() - grid_stamp_).seconds();
  return elapsed(scan_received_) <= navigation_limits_.input_timeout &&
         elapsed(grid_received_) <= navigation_limits_.input_timeout &&
         scan_age >= -0.01 && scan_age <= navigation_limits_.input_timeout &&
         grid_age >= -0.01 && grid_age <= navigation_limits_.input_timeout &&
         elapsed(safety_received_) <= 1.5;
}

bool MissionNode::inputs_ready() const {
  if (!navigation_->ready() || !clock_seen_ || !fresh_localization()) return false;
  if (!require_safety_) return true;
  // До первой команды monitor закономерно сообщает NOT_READY/STALE_CMD.
  return safety_inputs_fresh() &&
         (safety_reason_ == "OK" || safety_reason_ == "NOT_READY" || safety_reason_ == "STALE_CMD");
}

BT::NodeStatus MissionNode::wait_ready() {
  if (inputs_ready()) return BT::NodeStatus::SUCCESS;
  if (elapsed(episode_started_) > readiness_timeout_) {
    begin_cancel("readiness_timeout");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void MissionNode::release_jobs() {
  std::vector<std::string> previous;
  for (const auto &job : queue_.jobs()) {
    if (job.state == JobState::Unreleased && job.release_time <= episode_time()) previous.push_back(job.id);
  }
  queue_.release(episode_time());
  const auto active = active_job_;
  for (const auto &id : previous) {
    active_job_ = id;
    record_event("released");
  }
  active_job_ = active;
}

DispatchContext MissionNode::dispatch_context() const {
  DispatchContext context;
  context.now = episode_time();
  context.robot_x = localization_.pose.pose.position.x;
  context.robot_y = localization_.pose.pose.position.y;
  context.navigation_failures = navigation_failures_;
  if (!active_job_.empty()) {
    for (const auto &job : queue_.jobs()) {
      if (job.id == active_job_) context.loaded = job.state == JobState::Picked;
    }
  }
  return context;
}

BT::NodeStatus MissionNode::select_job() {
  if (!inputs_ready()) return wait_ready();
  if (episode_time() < wait_until_) return BT::NodeStatus::RUNNING;
  auto candidates = queue_.candidates();
  if (candidates.empty()) return BT::NodeStatus::RUNNING;
  const auto context = dispatch_context();
  const auto travel = estimate_travel(scenario_, candidates, context);
  const auto started = Clock::now();
  const auto decision = choose_action(candidates, travel, context, model_.get(),
                                      method_ == "learned", congestion_feature_);
  record_event("decision", "latency_s=" + std::to_string(elapsed(started)));
  if (decision.fallback) {
    ++fallback_count_;
    record_event("fallback", model_error_.empty() ? decision.reason : model_error_);
  }
  if (decision.action == 4) {
    wait_until_ = episode_time() + 1.0;
    record_event("wait");
    return BT::NodeStatus::RUNNING;
  }
  if (decision.action < 0 || static_cast<std::size_t>(decision.action) >= candidates.size()) {
    throw std::runtime_error("dispatcher returned an invalid slot");
  }
  active_job_ = candidates.at(static_cast<std::size_t>(decision.action))->id;
  queue_.mark_active(active_job_);
  attempts_ = 0;
  record_event("selected");
  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped MissionNode::target_pose(const std::string &station_id) const {
  const auto &station = station_by_id(scenario_, station_id);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = now();
  pose.pose.position.x = station.x;
  pose.pose.position.y = station.y;
  pose.pose.orientation.z = std::sin(station.yaw / 2.0);
  pose.pose.orientation.w = std::cos(station.yaw / 2.0);
  return pose;
}

BT::NodeStatus MissionNode::navigate(bool pickup) {
  auto &job = queue_.find(active_job_);
  const auto target = pickup ? job.pickup : job.dropoff;
  if (navigation_->state() == NavigationClient::State::Idle) {
    if (episode_time() < retry_at_) return BT::NodeStatus::RUNNING;
    navigation_target_ = target;
    ++attempts_;
    navigation_->start(target_pose(target));
  }
  if (navigation_->state() == NavigationClient::State::Succeeded) {
    navigation_->clear();
    attempts_ = 0;
    service_started_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  if (navigation_->state() != NavigationClient::State::Failed) return BT::NodeStatus::RUNNING;
  ++navigation_failures_;
  const auto reason = navigation_->reason();
  if (!navigation_->safe_to_retry()) {
    fail_job(reason);
    state_ = State::Fault;
    record_event("episode_fault", reason);
    return BT::NodeStatus::FAILURE;
  }
  // Две попытки всего на этап; каждая повторно запрашивает путь.
  if (attempts_ < 2) {
    record_event("recovery", reason, target);
    navigation_->clear();
    retry_at_ = episode_time() + 1.0;
    return BT::NodeStatus::RUNNING;
  }
  const bool loaded = job.state == JobState::Picked;
  fail_job(reason);
  if (loaded) {
    state_ = State::Fault;
    record_event("episode_fault", "loaded_delivery_failed: " + reason);
  } else navigation_->clear();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MissionNode::service(bool pickup) {
  auto &job = queue_.find(active_job_);
  if (!navigation_->stopped()) {
    begin_cancel("not_stopped_during_service");
    return BT::NodeStatus::FAILURE;
  }
  if (!service_started_) {
    service_started_ = true;
    service_until_ = episode_time() + (pickup ? job.pickup_service : job.dropoff_service);
    record_event(pickup ? "pickup_service" : "dropoff_service");
  }
  if (episode_time() < service_until_) return BT::NodeStatus::RUNNING;
  if (pickup) {
    queue_.mark_picked(active_job_, episode_time());
    record_event("picked");
  }
  service_started_ = false;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionNode::complete_job() {
  queue_.mark_delivered(active_job_, episode_time());
  record_event("delivered");
  active_job_.clear();
  navigation_target_.clear();
  return BT::NodeStatus::SUCCESS;
}

void MissionNode::fail_job(const std::string &reason) {
  if (active_job_.empty()) return;
  queue_.mark_failed(active_job_, reason);
  record_event("failed", reason, navigation_target_);
  active_job_.clear();
}

void MissionNode::begin_cancel(const std::string &reason, bool reset_after) {
  if (state_ == State::Cancelling) return;
  cancel_reason_ = reason;
  reset_after_cancel_ = reset_after;
  state_ = State::Cancelling;
  navigation_->cancel(reason);
  record_event("cancel_requested", reason);
}

void MissionNode::finish_cancel() {
  if (navigation_->state() != NavigationClient::State::Failed) return;
  tree_.haltTree();
  const bool safe = navigation_->safe_to_retry();
  fail_job(cancel_reason_);
  state_ = safe ? State::Cancelled : State::Fault;
  record_event(safe ? "episode_cancelled" : "episode_fault",
               safe ? cancel_reason_ : navigation_->reason());
  if (reset_after_cancel_ && safe && !shutdown_requested_) reset_episode(autostart_);
}

void MissionNode::on_timer() {
  process_pending_inputs();
  navigation_->poll();
  if (state_ == State::Cancelling) finish_cancel();
  if (state_ == State::Running) {
    const double clock = now().seconds();
    if (clock < last_clock_ - 0.001) {
      begin_cancel("time_reset", true);
    } else if (clock > last_clock_) {
      last_clock_progress_ = Clock::now();
      if (!clock_seen_) epoch_ = clock;
      clock_seen_ = true;
    }
    last_clock_ = clock;
    if (elapsed(last_clock_progress_) > clock_timeout_) begin_cancel("frozen_clock");
    if (state_ == State::Running && clock_seen_) {
      release_jobs();
      if (!active_job_.empty() && (!safety_inputs_fresh() || !fresh_localization())) {
        RCLCPP_ERROR(get_logger(),
            "stale navigation input: pose age=%.3f/wall=%.3f, scan age=%.3f/wall=%.3f, "
            "grid age=%.3f/wall=%.3f, safety heartbeat age=%.3f",
            (now() - rclcpp::Time(localization_.header.stamp)).seconds(),
            elapsed(localization_received_), (now() - scan_stamp_).seconds(),
            elapsed(scan_received_), (now() - grid_stamp_).seconds(),
            elapsed(grid_received_), elapsed(safety_received_));
        begin_cancel("stale_navigation_input");
      }
      if (state_ != State::Running) return;
      if (episode_time() >= scenario_.horizon) begin_cancel("episode_timeout");
      else if (queue_.all_terminal()) {
        const bool failed = std::any_of(queue_.jobs().begin(), queue_.jobs().end(),
            [](const Job &job) { return job.state == JobState::Failed; });
        state_ = failed ? State::Fault : State::Completed;
        record_event(failed ? "episode_failed" : "episode_completed",
                     failed ? "unfinished_deliveries" : "");
      } else {
        try {
          const auto result = tree_.tickOnce();
          if (result != BT::NodeStatus::RUNNING) tree_.haltTree();
        } catch (const std::exception &error) {
          begin_cancel(std::string("mission_exception: ") + error.what());
        }
      }
    }
  }
  if (elapsed(last_status_) >= 1.0) publish_status();
}

std::string MissionNode::state_name() const {
  switch (state_) {
    case State::Idle: return "idle";
    case State::Running: return "running";
    case State::Cancelling: return "cancelling";
    case State::Completed: return "completed";
    case State::Cancelled: return "cancelled";
    case State::Fault: return "fault";
  }
  return "fault";
}

void MissionNode::record_event(const std::string &event, const std::string &reason,
                                const std::string &target) {
  if (events_.is_open()) {
    events_ << csv(episode_id_) << ',' << csv(active_job_) << ',' << std::setprecision(12)
            << episode_time() << ',' << csv(event) << ',' << csv(reason) << ',' << csv(target) << '\n';
    events_.flush();
    if (!events_) throw std::runtime_error("event log write failed");
  }
  RCLCPP_INFO(get_logger(), "episode=%s job=%s event=%s reason=%s", episode_id_.c_str(),
              active_job_.c_str(), event.c_str(), reason.c_str());
  publish_status();
}

void MissionNode::publish_status() {
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "capstone_mission/mission";
  status.message = state_name();
  status.level = state_ == State::Fault ? status.ERROR : status.OK;
  auto add = [&status](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    status.values.push_back(pair);
  };
  int released = 0, delivered = 0, failed = 0;
  for (const auto &job : queue_.jobs()) {
    if (job.state != JobState::Unreleased) ++released;
    if (job.state == JobState::Delivered) ++delivered;
    if (job.state == JobState::Failed) ++failed;
  }
  add("released", std::to_string(released));
  add("delivered", std::to_string(delivered));
  add("failed", std::to_string(failed));
  add("unfinished", std::to_string(released - delivered));
  add("episode_time", std::to_string(episode_time()));
  add("episode_id", episode_id_);
  add("job_id", active_job_);
  add("method", method_);
  add("fallback_count", std::to_string(fallback_count_));
  add("navigation_failures", std::to_string(navigation_failures_));
  add("distance_to_goal", navigation_ ? std::to_string(navigation_->distance_to_goal()) : "0");
  message.status.push_back(status);
  status_pub_->publish(message);
  last_status_ = Clock::now();
}

void MissionNode::request_shutdown() {
  shutdown_requested_ = true;
  if (state_ == State::Running) begin_cancel("shutdown");
}

bool MissionNode::shutdown_ready() const {
  return state_ != State::Running && state_ != State::Cancelling;
}

}  // namespace capstone_mission
