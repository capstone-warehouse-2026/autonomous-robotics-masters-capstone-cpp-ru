#include "capstone_safety/safety_monitor_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/exceptions.h>

namespace capstone_safety {

std::vector<Point> scan_to_points(const sensor_msgs::msg::LaserScan &scan, const Pose2D &sensor_in_base) {
  std::vector<Point> points;
  points.reserve(scan.ranges.size());
  const double c = std::cos(sensor_in_base.yaw);
  const double s = std::sin(sensor_in_base.yaw);
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double lx = r * std::cos(a);
    const double ly = r * std::sin(a);
    // Rotate by the LiDAR yaw and shift by its position in base_link.
    points.push_back({sensor_in_base.x + c * lx - s * ly, sensor_in_base.y + s * lx + c * ly});
  }
  return points;
}

// ------------------------------------------------------------------ construction and parameters

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("safety_monitor", options),
      method_(declare_parameter<std::string>("method", "classical")),
      base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
      autostart_(declare_parameter<bool>("autostart", true)),
      rate_hz_(declare_parameter<double>("rate_hz", 20.0)),
      core_(declare_core_params()) {  // invalid thresholds throw already here
  // The monitor is the same for A and B; method is kept for a uniform launch configuration.
  if (method_ != "classical" && method_ != "learned") {
    throw std::invalid_argument("method must be 'classical' or 'learned', got '" + method_ + "'");
  }
  if (!(rate_hz_ > 0.0)) throw std::invalid_argument("rate_hz must be > 0");
  RCLCPP_INFO(get_logger(), "created (unconfigured), autostart=%s", autostart_ ? "true" : "false");
}

Params SafetyMonitorNode::declare_core_params() {
  const Params d;  // defaults from safety_core.hpp
  Params p;
  p.v_max = declare_parameter<double>("v_max", d.v_max);
  p.w_max = declare_parameter<double>("w_max", d.w_max);
  p.limit_tol = declare_parameter<double>("limit_tol", d.limit_tol);
  p.a_brake = declare_parameter<double>("a_brake", d.a_brake);
  p.t_latency = declare_parameter<double>("t_latency", d.t_latency);
  p.robot_radius = declare_parameter<double>("robot_radius", d.robot_radius);
  p.margin = declare_parameter<double>("margin", d.margin);
  p.path_step = declare_parameter<double>("path_step", d.path_step);
  p.v_eps = declare_parameter<double>("v_eps", d.v_eps);
  p.cmd_timeout_sim = declare_parameter<double>("cmd_timeout_sim", d.cmd_timeout_sim);
  p.cmd_timeout_steady = declare_parameter<double>("cmd_timeout_steady", d.cmd_timeout_steady);
  p.sensor_timeout_sim = declare_parameter<double>("sensor_timeout_sim", d.sensor_timeout_sim);
  p.time_jump_tol = declare_parameter<double>("time_jump_tol", d.time_jump_tol);
  return p;  // the SafetyCore constructor validates the values
}

Params SafetyMonitorNode::read_core_params() {
  Params p;
  p.v_max = get_parameter("v_max").as_double();
  p.w_max = get_parameter("w_max").as_double();
  p.limit_tol = get_parameter("limit_tol").as_double();
  p.a_brake = get_parameter("a_brake").as_double();
  p.t_latency = get_parameter("t_latency").as_double();
  p.robot_radius = get_parameter("robot_radius").as_double();
  p.margin = get_parameter("margin").as_double();
  p.path_step = get_parameter("path_step").as_double();
  p.v_eps = get_parameter("v_eps").as_double();
  p.cmd_timeout_sim = get_parameter("cmd_timeout_sim").as_double();
  p.cmd_timeout_steady = get_parameter("cmd_timeout_steady").as_double();
  p.sensor_timeout_sim = get_parameter("sensor_timeout_sim").as_double();
  p.time_jump_tol = get_parameter("time_jump_tol").as_double();
  return p;
}

// ------------------------------------------------------------------ lifecycle

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_configure(const rclcpp_lifecycle::State &) {
  // Parameters may have changed between construction and configure (ros2 param set): read them again.
  try {
    base_frame_ = get_parameter("base_frame").as_string();
    rate_hz_ = get_parameter("rate_hz").as_double();
    if (!(rate_hz_ > 0.0)) throw std::invalid_argument("rate_hz must be > 0");
    core_ = SafetyCore(read_core_params());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    return CallbackReturn::FAILURE;  // the node stays unconfigured
  }
  sensor_pose_.reset();
  sensor_frame_.clear();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS as in docs/INTERFACES.md.
  cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/control/cmd_vel_raw", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](const geometry_msgs::msg::TwistStamped &msg) { on_cmd(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state/odom", rclcpp::QoS(5).reliable().durability_volatile(),
      [this](const nav_msgs::msg::Odometry &msg) { on_odom(msg); });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensors/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan &msg) { on_scan(msg); });

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(1).reliable().durability_volatile());
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10).reliable());

  RCLCPP_INFO(get_logger(), "configured: %.1f Hz, base frame '%s'", rate_hz_, base_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_activate(const rclcpp_lifecycle::State &) {
  // Forget everything received before activation: decisions use only data received after it.
  core_.reset();
  last_decision_ = Decision{};
  last_reason_.reset();
  ticks_ = 0;
  reason_counts_.clear();

  cmd_pub_->on_activate();
  diag_pub_->on_activate();

  // A wall (steady) timer, not a ROS timer: if /clock stops, the monitor keeps ticking and stops the
  // robot on the steady-time timeout.
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz_));
  control_timer_ = create_wall_timer(period, [this] { on_timer(); });
  diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_diagnostics(last_decision_); });

  RCLCPP_INFO(get_logger(), "active: /cmd_vel is published at %.1f Hz", rate_hz_);
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_deactivate(const rclcpp_lifecycle::State &) {
  if (control_timer_) control_timer_->cancel();
  if (diag_timer_) diag_timer_->cancel();
  control_timer_.reset();
  diag_timer_.reset();

  publish_stop();  // last zero; then silence, and the driver watchdog stops the robot
  cmd_pub_->on_deactivate();
  diag_pub_->on_deactivate();

  RCLCPP_WARN(get_logger(), "inactive: /cmd_vel is not published, driver watchdog holds the robot");
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_cleanup(const rclcpp_lifecycle::State &) {
  release_interfaces();
  RCLCPP_INFO(get_logger(), "cleaned up");
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_shutdown(const rclcpp_lifecycle::State &) {
  publish_stop();
  release_interfaces();
  RCLCPP_INFO(get_logger(), "shut down");
  return CallbackReturn::SUCCESS;
}

void SafetyMonitorNode::release_interfaces() {
  control_timer_.reset();
  diag_timer_.reset();
  cmd_sub_.reset();
  odom_sub_.reset();
  scan_sub_.reset();
  cmd_pub_.reset();
  diag_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
}

void SafetyMonitorNode::publish_stop() {
  if (!cmd_pub_ || !cmd_pub_->is_activated()) return;
  geometry_msgs::msg::TwistStamped zero;
  zero.header.stamp = now();
  zero.header.frame_id = base_frame_;
  cmd_pub_->publish(zero);
}

// ------------------------------------------------------------------ inputs

double SafetyMonitorNode::steady_now() { return steady_clock_.now().seconds(); }

void SafetyMonitorNode::on_cmd(const geometry_msgs::msg::TwistStamped &msg) {
  // A command in another frame is rejected: the monitor does not guess a transform.
  if (msg.header.frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "cmd_vel_raw in frame '%s', expected '%s': ignored",
                         msg.header.frame_id.c_str(), base_frame_.c_str());
    return;
  }
  last_in_v_ = msg.twist.linear.x;
  last_in_w_ = msg.twist.angular.z;

  Cmd c;
  c.t_sim = rclcpp::Time(msg.header.stamp).seconds();
  c.t_steady = steady_now();
  c.v = msg.twist.linear.x;
  c.w = msg.twist.angular.z;
  core_.on_cmd(c);
}

void SafetyMonitorNode::on_odom(const nav_msgs::msg::Odometry &msg) {
  Odom o;
  o.t_sim = rclcpp::Time(msg.header.stamp).seconds();
  o.t_steady = steady_now();
  o.v = msg.twist.twist.linear.x;
  core_.on_odom(o);
}

void SafetyMonitorNode::on_scan(const sensor_msgs::msg::LaserScan &msg) {
  Pose2D pose;
  // Without the LiDAR pose the scan is dropped: the core reports STALE_SCAN and stops the robot.
  if (!sensor_pose(msg.header.frame_id, pose)) return;

  Scan s;
  s.t_sim = rclcpp::Time(msg.header.stamp).seconds();
  s.t_steady = steady_now();
  s.points = scan_to_points(msg, pose);
  core_.on_scan(s);
}

bool SafetyMonitorNode::sensor_pose(const std::string &frame, Pose2D &out) {
  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "scan without frame_id: ignored");
    return false;
  }
  if (frame == base_frame_) {  // already in base_link
    out = Pose2D{};
    return true;
  }
  if (sensor_pose_ && frame == sensor_frame_) {
    out = *sensor_pose_;
    return true;
  }
  try {
    // The LiDAR is rigidly mounted, so the latest (static) transform is used.
    const auto tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
    const auto &q = tf.transform.rotation;
    Pose2D p;
    p.x = tf.transform.translation.x;
    p.y = tf.transform.translation.y;
    p.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    sensor_pose_ = p;
    sensor_frame_ = frame;
    out = p;
    RCLCPP_INFO(get_logger(), "laser '%s' in '%s': x=%.3f y=%.3f yaw=%.3f", frame.c_str(), base_frame_.c_str(), p.x,
                p.y, p.yaw);
    return true;
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "no TF %s <- %s: %s", base_frame_.c_str(), frame.c_str(),
                         e.what());
    return false;
  }
}

// ------------------------------------------------------------------ output

void SafetyMonitorNode::on_timer() {
  const Decision d = core_.step(now().seconds(), steady_now());
  ++ticks_;
  ++reason_counts_[d.reason];

  // Publish every tick, zeros included: a single last zero Twist is not enough.
  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = now();
  out.header.frame_id = base_frame_;
  out.twist.linear.x = d.v;
  out.twist.angular.z = d.w;
  cmd_pub_->publish(out);

  last_decision_ = d;
  if (!last_reason_ || *last_reason_ != d.reason) {  // a new reason is an event: log and report at once
    if (d.intervened()) {
      RCLCPP_WARN(get_logger(), "halt: %s", to_string(d.reason).c_str());
    } else {
      RCLCPP_INFO(get_logger(), "commands pass through");
    }
    last_reason_ = d.reason;
    publish_diagnostics(d);
  }
}

void SafetyMonitorNode::publish_diagnostics(const Decision &d) {
  if (!diag_pub_ || !diag_pub_->is_activated()) return;

  using diagnostic_msgs::msg::DiagnosticStatus;
  DiagnosticStatus st;
  st.name = "capstone_safety/safety_monitor";
  st.message = to_string(d.reason);
  if (d.reason == Reason::OK) {
    st.level = DiagnosticStatus::OK;
  } else if (d.reason == Reason::NOT_READY) {
    st.level = DiagnosticStatus::WARN;
  } else {
    st.level = DiagnosticStatus::ERROR;
  }

  auto add = [&st](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    st.values.push_back(kv);
  };
  add("reason", to_string(d.reason));
  add("latched", core_.latched() ? "true" : "false");
  add("in_v", std::to_string(last_in_v_));
  add("in_w", std::to_string(last_in_w_));
  add("out_v", std::to_string(d.v));
  add("out_w", std::to_string(d.w));
  add("ticks", std::to_string(ticks_));
  for (const auto &[reason, count] : reason_counts_) add("count_" + to_string(reason), std::to_string(count));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(st);
  diag_pub_->publish(array);
}

}  // namespace capstone_safety
