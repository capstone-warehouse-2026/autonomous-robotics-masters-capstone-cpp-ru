#include "capstone_nav2/contracts_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tf2/exceptions.hpp>

namespace capstone_nav2 {

ContractsAdapter::ContractsAdapter(const rclcpp::NodeOptions &options)
    : rclcpp::Node("nav2_contracts", options) {
  read_parameters();
  create_ros_interfaces();
  update_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                  [this] { request_localization_update(); });
  scan_retry_timer_ = create_wall_timer(std::chrono::milliseconds(20),
      [this] { process_pending_scan(); process_pending_localization(); });
}

void ContractsAdapter::read_parameters() {
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  grid_frame_ = declare_parameter<std::string>("grid_frame", "odom");
  grid_parameters_.size = declare_parameter<double>("grid_size", 8.0);
  grid_parameters_.resolution = declare_parameter<double>("grid_resolution", 0.05);
  max_input_age_ = declare_parameter<double>("max_input_age", 0.3);
  initial_x_ = declare_parameter<double>("initial_x", -3.0);
  initial_y_ = declare_parameter<double>("initial_y", 0.0);
  initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
  if (base_frame_.empty() || grid_frame_.empty() || !std::isfinite(max_input_age_) ||
      max_input_age_ <= 0 || !std::isfinite(grid_parameters_.size) ||
      !std::isfinite(grid_parameters_.resolution) || grid_parameters_.size <= 0 ||
      grid_parameters_.resolution <= 0 || grid_parameters_.size / grid_parameters_.resolution > 1000 ||
      !std::isfinite(initial_x_) || !std::isfinite(initial_y_) || !std::isfinite(initial_yaw_))
    throw std::invalid_argument("invalid contracts adapter parameters");
}

void ContractsAdapter::create_ros_interfaces() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "control/cmd_vel_raw", rclcpp::QoS(1).reliable());
  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "perception/obstacles", rclcpp::QoS(1).reliable());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", rclcpp::QoS(5).reliable());
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1).reliable());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "nav2/cmd_vel", rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::Twist &command) { on_command(command); });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "sensors/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan &scan) { on_scan(scan); });
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::QoS(5).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped &pose) { on_localization(pose); });
  update_client_ = create_client<std_srvs::srv::Empty>("request_nomotion_update");
  clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "clock", rclcpp::QoS(10).best_effort(),
      [this](const rosgraph_msgs::msg::Clock &clock) { on_clock(clock); });
}

void ContractsAdapter::on_command(const geometry_msgs::msg::Twist &command) {
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.frame_id = base_frame_;
  stamped.header.stamp = now();
  const bool finite = std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
      std::isfinite(command.linear.z) && std::isfinite(command.angular.x) &&
      std::isfinite(command.angular.y) && std::isfinite(command.angular.z);
  if (initial_pose_pending_ || reset_confirmation_pending_) {
    publish_diagnostic(1, "WAITING_FOR_AMCL_RESET");
  } else if (!finite || std::abs(command.linear.y) > 1e-6 || std::abs(command.linear.z) > 1e-6 ||
      std::abs(command.angular.x) > 1e-6 || std::abs(command.angular.y) > 1e-6) {
    publish_diagnostic(2, "INVALID_NAV2_COMMAND");
  } else {
    stamped.twist = command;
  }
  // Присваивается stamp получения новой команды Twist; повторной публикации старой нет.
  command_pub_->publish(stamped);
}

void ContractsAdapter::on_scan(const sensor_msgs::msg::LaserScan &scan) {
  // Предыдущий scan мог дождаться TF как раз к приходу следующего: сначала сохраняем
  // его результат, иначе одинаковая фаза scan/TF способна бесконечно вытеснять данные.
  process_pending_scan();
  pending_scan_ = scan;
  scan_received_ = Clock::now();
  // Запрос по каждому scan сохраняет частоту измерений и в ускоренной симуляции.
  request_localization_update();
}

void ContractsAdapter::process_pending_scan() {
  if (!pending_scan_) return;
  const auto &scan = *pending_scan_;
  if (scan.header.stamp.sec < 0 || scan.header.stamp.nanosec >= 1000000000U) {
    publish_diagnostic(2, "INVALID_SCAN_STAMP");
    pending_scan_.reset();
    return;
  }
  const double age = (now() - rclcpp::Time(scan.header.stamp)).seconds();
  const double waiting = std::chrono::duration<double>(Clock::now() - scan_received_).count();
  if (age < -0.01 && age >= -max_input_age_ && waiting <= max_input_age_) {
    // /clock и scan идут разными DDS очередями: ждём clock, не обновляя stamp.
    return;
  }
  if (age < -0.01 || age > max_input_age_ || waiting > max_input_age_ ||
      scan.header.frame_id.empty()) {
    publish_diagnostic(2, "STALE_SCAN");
    pending_scan_.reset();
    return;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(grid_frame_, scan.header.frame_id,
                                                       rclcpp::Time(scan.header.stamp));
    const auto &rotation = transform.transform.rotation;
    const double yaw = std::atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                                 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z));
    auto grid = project_scan(scan, {transform.transform.translation.x,
                                   transform.transform.translation.y, yaw}, grid_parameters_);
    grid.header.frame_id = grid_frame_;
    grid_pub_->publish(grid);
    pending_scan_.reset();
    publish_diagnostic(0, "SCAN_GRID_READY");
  } catch (const tf2::TransformException &error) {
    publish_diagnostic(1, "SCAN_TF_UNAVAILABLE");
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "scan TF: %s", error.what());
  } catch (const std::invalid_argument &error) {
    publish_diagnostic(2, "INVALID_SCAN");
    pending_scan_.reset();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "scan: %s", error.what());
  }
}

void ContractsAdapter::on_localization(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
  process_pending_localization();
  pending_pose_ = pose;
  pose_received_ = Clock::now();
  process_pending_localization();
}

void ContractsAdapter::process_pending_localization() {
  if (!pending_pose_) return;
  const auto &pose = *pending_pose_;
  if (pose.header.stamp.sec < 0 || pose.header.stamp.nanosec >= 1000000000U) {
    publish_diagnostic(2, "INVALID_POSE_STAMP");
    pending_pose_.reset();
    return;
  }
  const double age = (now() - rclcpp::Time(pose.header.stamp)).seconds();
  const double waiting = std::chrono::duration<double>(Clock::now() - pose_received_).count();
  if (age < -0.01 && age >= -max_input_age_ && waiting <= max_input_age_) return;
  if (initial_pose_pending_ || rclcpp::Time(pose.header.stamp).seconds() <= initial_pose_stamp_) {
    pending_pose_.reset();
    return;
  }
  const auto &position = pose.pose.pose.position;
  const auto &rotation = pose.pose.pose.orientation;
  const double norm = rotation.x * rotation.x + rotation.y * rotation.y +
                      rotation.z * rotation.z + rotation.w * rotation.w;
  const bool covariance_valid = std::all_of(pose.pose.covariance.begin(), pose.pose.covariance.end(),
                                           [](double value) { return std::isfinite(value); });
  // AMCL с вырожденной начальной выборкой даёт -1e-15 вместо нулевой дисперсии.
  // Допуск только для погрешности округления; исходная ковариация не переписывается.
  constexpr double covariance_roundoff = 1e-12;
  if (pose.header.frame_id != "map" || age < -0.01 || age > max_input_age_ || waiting > max_input_age_ ||
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 || !covariance_valid ||
      pose.pose.covariance[0] < -covariance_roundoff ||
      pose.pose.covariance[7] < -covariance_roundoff ||
      pose.pose.covariance[35] < -covariance_roundoff) {
    publish_diagnostic(2, "INVALID_AMCL_POSE");
    pending_pose_.reset();
    return;
  }
  if (reset_confirmation_pending_) {
    const double yaw = std::atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                                  1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z));
    if (std::hypot(position.x - initial_x_, position.y - initial_y_) > 0.5 ||
        std::abs(std::remainder(yaw - initial_yaw_, 2 * std::acos(-1.0))) > 0.3) {
      publish_diagnostic(1, "WAITING_FOR_AMCL_RESET");
      if (now().seconds() - initial_pose_stamp_ > 1.0) initial_pose_pending_ = true;
      pending_pose_.reset();
      return;
    }
    reset_confirmation_pending_ = false;
  }
  pose_pub_->publish(pose);
  pending_pose_.reset();
}

void ContractsAdapter::request_localization_update() {
  // TF и scan проходят разные DDS очереди; сохраняется только самый новый скан.
  process_pending_scan();
  if (initial_pose_pending_ && !restore_initial_pose()) return;
  if (update_request_) {
    if (Clock::now() - update_requested_ > std::chrono::seconds(1)) {
      update_client_->remove_pending_request(*update_request_);
      update_request_.reset();
      publish_diagnostic(2, "AMCL_UPDATE_TIMEOUT");
    }
    return;
  }
  if (!update_client_->service_is_ready()) return;
  // AMCL обычно публикует только после движения. Запрос вызывает НОВОЕ измерение скана,
  // поэтому готовность G5 не требует подделывать свежесть неподвижного робота.
  update_requested_ = Clock::now();
  const auto generation = request_generation_;
  const auto result = update_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
      [this, generation](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        if (generation == request_generation_) update_request_.reset();
      });
  update_request_ = result.request_id;
}

void ContractsAdapter::on_clock(const rosgraph_msgs::msg::Clock &clock) {
  const double stamp = clock.clock.sec + clock.clock.nanosec * 1e-9;
  if (stamp + 0.001 < last_clock_) {
    initial_pose_pending_ = true;
    reset_confirmation_pending_ = true;
    pending_scan_.reset();
    pending_pose_.reset();
    ++request_generation_;
    if (update_request_) update_client_->remove_pending_request(*update_request_);
    update_request_.reset();
    publish_diagnostic(1, "SIMULATION_RESET");
  }
  last_clock_ = stamp;
}

bool ContractsAdapter::restore_initial_pose() {
  if (now().seconds() < 0.1 || initial_pose_pub_->get_subscription_count() == 0) return false;
  if (!tf_buffer_->canTransform(grid_frame_, base_frame_, now())) return false;
  geometry_msgs::msg::PoseWithCovarianceStamped initial;
  initial.header.frame_id = "map";
  initial.header.stamp = now();
  initial_pose_stamp_ = now().seconds();
  initial.pose.pose.position.x = initial_x_;
  initial.pose.pose.position.y = initial_y_;
  initial.pose.pose.orientation.z = std::sin(initial_yaw_ / 2.0);
  initial.pose.pose.orientation.w = std::cos(initial_yaw_ / 2.0);
  initial.pose.covariance[0] = initial.pose.covariance[7] = 0.01;
  initial.pose.covariance[35] = 0.01;
  // После reset это новая начальная гипотеза из сценария; ground truth не подписывается.
  initial_pose_pub_->publish(initial);
  initial_pose_pending_ = false;
  publish_diagnostic(0, "INITIAL_POSE_RESTORED");
  return true;
}

void ContractsAdapter::publish_diagnostic(unsigned char level, const std::string &reason) {
  if (last_diagnostic_ == reason) return;
  last_diagnostic_ = reason;
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "capstone_nav2/contracts";
  status.hardware_id = "nav2_baseline";
  status.level = level;
  status.message = reason;
  message.status.push_back(status);
  diagnostics_pub_->publish(message);
}

}  // namespace capstone_nav2
