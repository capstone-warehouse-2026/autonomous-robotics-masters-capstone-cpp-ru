#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "capstone_webots_bridge/odometry.hpp"

namespace capstone_webots_bridge {
namespace {
constexpr double kWheelRadius = 0.0985;
constexpr double kWheelSeparation = 0.4044;
constexpr double kLaserHeight = 0.25;
using Clock = std::chrono::steady_clock;

WbDeviceTag required_device(const char *name) {
  const auto tag = wb_robot_get_device(name);
  if (!tag) throw std::runtime_error(std::string("Webots device missing: ") + name);
  return tag;
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(yaw * 0.5);
  result.w = std::cos(yaw * 0.5);
  return result;
}
}  // namespace

class WebotsBridge : public rclcpp::Node {
 public:
  WebotsBridge() : rclcpp::Node("webots_bridge"), odometry_(kWheelRadius, kWheelSeparation) {
    publish_clock_ = declare_parameter("publish_clock", true);
    enable_reset_ = declare_parameter("enable_reset", true);
    absolute_time_ = declare_parameter("absolute_time", false);
    if (enable_reset_ && (absolute_time_ || !publish_clock_)) {
      throw std::invalid_argument("enable_reset requires an independent clock publisher and epoch");
    }
    configure_devices();
    create_ros_interfaces();
    publish_static_transform();
    RCLCPP_INFO(get_logger(), "Webots physical bridge ready: wheel encoders, LiDAR, step=%d ms", step_ms_);
  }

  int step_ms() const { return step_ms_; }

  void before_step() {
    if (reset_header_ && !reset_applied_) apply_reset();
    const double wall_age = std::chrono::duration<double>(Clock::now() - command_received_).count();
    const double command_stamp = rclcpp::Time(command_.header.stamp).seconds();
    const bool valid = command_received_valid_ && command_.header.frame_id == "base_link" &&
        command_is_fresh(command_stamp, simulation_time(), wall_age,
                         command_.twist.linear.x, command_.twist.angular.z);
    double left = 0.0, right = 0.0;
    if (valid) {
      left = (command_.twist.linear.x - command_.twist.angular.z * kWheelSeparation * 0.5) / kWheelRadius;
      right = (command_.twist.linear.x + command_.twist.angular.z * kWheelSeparation * 0.5) / kWheelRadius;
      const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)) / max_wheel_speed_);
      left /= scale;
      right /= scale;
    }
    wb_motor_set_velocity(left_motor_, left);
    wb_motor_set_velocity(right_motor_, right);
  }

  void after_step() {
    const auto stamp = rclcpp::Time(static_cast<int64_t>(std::llround(simulation_time() * 1e9)));
    if (publish_clock_) publish_clock(stamp);
    publish_odometry(stamp);
    if (++scan_steps_ >= scan_period_steps_) {
      scan_steps_ = 0;
      publish_scan(stamp);
    }
    publish_evaluation(stamp);
    if (reset_header_ && reset_applied_) {
      // Ответ приходит после физического шага, новых энкодеров и публикации нового clock.
      std_srvs::srv::Trigger::Response response;
      response.success = true;
      response.message = "robot physics and encoder odometry reset; episode clock restarted";
      reset_service_->send_response(*reset_header_, response);
      reset_header_.reset();
      reset_applied_ = false;
    }
  }

  void stop_motors() {
    wb_motor_set_velocity(left_motor_, 0.0);
    wb_motor_set_velocity(right_motor_, 0.0);
  }

 private:
  double simulation_time() const { return std::max(0.0, wb_robot_get_time() - epoch_); }

  void configure_devices() {
    step_ms_ = static_cast<int>(std::lround(wb_robot_get_basic_time_step()));
    if (step_ms_ <= 0 || 100 % step_ms_ != 0) {
      throw std::runtime_error("world basicTimeStep must divide 100 ms");
    }
    scan_period_steps_ = 100 / step_ms_;
    left_motor_ = required_device("left_motor");
    right_motor_ = required_device("right_motor");
    left_encoder_ = required_device("left_encoder");
    right_encoder_ = required_device("right_encoder");
    lidar_ = required_device("lidar");
    self_ = wb_supervisor_node_get_self();
    if (!self_) throw std::runtime_error("bridge robot must be a Webots Supervisor");
    wb_supervisor_node_save_state(self_, "episode_initial_pose");
    wb_motor_set_position(left_motor_, INFINITY);
    wb_motor_set_position(right_motor_, INFINITY);
    stop_motors();
    max_wheel_speed_ = std::min(wb_motor_get_max_velocity(left_motor_), wb_motor_get_max_velocity(right_motor_));
    wb_position_sensor_enable(left_encoder_, step_ms_);
    wb_position_sensor_enable(right_encoder_, step_ms_);
    wb_lidar_enable(lidar_, 100);
    if (wb_lidar_get_number_of_layers(lidar_) != 1) {
      throw std::runtime_error("bridge requires a single layer planar LiDAR");
    }
    // У нескольких синхронных контроллеров общий отсчёт — время физического мира.
    epoch_ = absolute_time_ ? 0.0 : wb_robot_get_time();
  }

  void create_ros_interfaces() {
    if (publish_clock_) {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("clock", rclcpp::ClockQoS());
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("state/odom", rclcpp::QoS(5).reliable());
    wheel_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensors/wheel_odom", rclcpp::SensorDataQoS());
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("sensors/scan", rclcpp::SensorDataQoS());
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("evaluation/ground_truth", rclcpp::QoS(10).reliable());
    contact_pub_ = create_publisher<std_msgs::msg::Bool>("evaluation/contact", rclcpp::QoS(10).reliable());
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", rclcpp::QoS(1).reliable(),
        [this](const geometry_msgs::msg::TwistStamped &message) {
          command_ = message;
          command_received_ = Clock::now();
          command_received_valid_ = true;
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>("simulation/reset",
        [this](std::shared_ptr<rmw_request_id_t> header,
               std::shared_ptr<std_srvs::srv::Trigger::Request>) {
          if (!enable_reset_) {
            std_srvs::srv::Trigger::Response response;
            response.success = false;
            response.message = "individual reset disabled: restart the complete fleet world";
            reset_service_->send_response(*header, response);
            return;
          }
          if (reset_header_) {
            std_srvs::srv::Trigger::Response response;
            response.success = false;
            response.message = "another reset is pending";
            reset_service_->send_response(*header, response);
            return;
          }
          reset_header_ = std::move(header);
        });
  }

  void publish_static_transform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "lidar_link";
    transform.transform.translation.z = kLaserHeight;
    transform.transform.rotation.w = 1.0;
    static_tf_pub_->sendTransform(transform);
  }

  void publish_clock(const rclcpp::Time &stamp) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = stamp;
    clock_pub_->publish(clock);
  }

  void publish_odometry(const rclcpp::Time &stamp) {
    const bool valid = odometry_.update(wb_position_sensor_get_value(left_encoder_),
        wb_position_sensor_get_value(right_encoder_), step_ms_ / 1000.0);
    if (!valid) {
      command_received_valid_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "invalid wheel encoder sample; stopping");
      return;
    }
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = odometry_.x;
    message.pose.pose.position.y = odometry_.y;
    message.pose.pose.orientation = yaw_quaternion(odometry_.yaw);
    message.twist.twist.linear.x = odometry_.velocity;
    message.twist.twist.angular.z = odometry_.yaw_rate;
    message.pose.covariance[0] = message.pose.covariance[7] = 0.0001 + 0.001 * odometry_.distance;
    message.pose.covariance[35] = 0.0001 + 0.0005 * odometry_.distance;
    message.pose.covariance[14] = message.pose.covariance[21] = message.pose.covariance[28] = 1e6;
    message.twist.covariance[0] = 0.001;
    message.twist.covariance[35] = 0.001;
    odom_pub_->publish(message);
    wheel_pub_->publish(message);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = odometry_.x;
    transform.transform.translation.y = odometry_.y;
    transform.transform.rotation = message.pose.pose.orientation;
    tf_pub_->sendTransform(transform);
  }

  void publish_scan(const rclcpp::Time &stamp) {
    const auto *ranges = wb_lidar_get_layer_range_image(lidar_, 0);
    if (!ranges) return;
    const int count = wb_lidar_get_horizontal_resolution(lidar_);
    if (count < 2) throw std::runtime_error("invalid LiDAR resolution");
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = stamp;
    message.header.frame_id = "lidar_link";
    message.angle_min = -wb_lidar_get_fov(lidar_) * 0.5;
    message.angle_max = wb_lidar_get_fov(lidar_) * 0.5;
    message.angle_increment = wb_lidar_get_fov(lidar_) / (count - 1);
    message.scan_time = 0.1;
    // Fixed lidar рендерит весь слой одновременно, без последовательного вращения.
    message.time_increment = 0.0;
    message.range_min = wb_lidar_get_min_range(lidar_);
    message.range_max = wb_lidar_get_max_range(lidar_);
    message.ranges.resize(static_cast<std::size_t>(count));
    // Webots хранит лучи слева направо; ROS требует возрастающий угол против часовой стрелки.
    std::reverse_copy(ranges, ranges + count, message.ranges.begin());
    scan_pub_->publish(message);
  }

  void publish_evaluation(const rclcpp::Time &stamp) {
    // Единственное место чтения истинной позы: эти данные не участвуют в odom/TF/Nav2.
    const double *position = wb_supervisor_node_get_position(self_);
    const double *rotation = wb_supervisor_node_get_orientation(self_);
    const double *velocity = wb_supervisor_node_get_velocity(self_);
    const double yaw = std::atan2(rotation[3], rotation[0]);
    nav_msgs::msg::Odometry truth;
    truth.header.stamp = stamp;
    truth.header.frame_id = "map";
    truth.child_frame_id = "base_link";
    truth.pose.pose.position.x = position[0];
    truth.pose.pose.position.y = position[1];
    truth.pose.pose.position.z = position[2];
    truth.pose.pose.orientation = yaw_quaternion(yaw);
    truth.twist.twist.linear.x = std::cos(yaw) * velocity[0] + std::sin(yaw) * velocity[1];
    truth.twist.twist.angular.z = velocity[5];
    truth_pub_->publish(truth);
    int count = 0;
    const auto *points = wb_supervisor_node_get_contact_points(self_, true, &count);
    std_msgs::msg::Bool contact;
    // Колёса и опоры всегда касаются пола z=0. Контакты выше 4 см — реальные столкновения
    // со стенами/стеллажами этого мира, а не подсказка для управления.
    for (int i = 0; i < count; ++i) {
      if (points[i].point[2] > 0.04) contact.data = true;
    }
    contact_pub_->publish(contact);
  }

  void apply_reset() {
    stop_motors();
    command_received_valid_ = false;
    command_ = geometry_msgs::msg::TwistStamped{};
    wb_supervisor_node_load_state(self_, "episode_initial_pose");
    wb_supervisor_node_reset_physics(self_);
    wb_motor_set_position(left_motor_, INFINITY);
    wb_motor_set_position(right_motor_, INFINITY);
    stop_motors();
    odometry_.reset();
    epoch_ = wb_robot_get_time();
    scan_steps_ = 0;
    reset_applied_ = true;
    publish_clock(rclcpp::Time(0));
    RCLCPP_INFO(get_logger(), "Reset requested: restored physical spawn and restarted episode clock");
  }

  int step_ms_{20}, scan_period_steps_{5}, scan_steps_{0};
  double epoch_{0}, max_wheel_speed_{0};
  WbDeviceTag left_motor_{0}, right_motor_{0}, left_encoder_{0}, right_encoder_{0}, lidar_{0};
  WbNodeRef self_{nullptr};
  WheelOdometry odometry_;
  geometry_msgs::msg::TwistStamped command_;
  Clock::time_point command_received_{};
  bool command_received_valid_{false}, reset_applied_{false};
  bool publish_clock_{true}, enable_reset_{true}, absolute_time_{false};
  std::shared_ptr<rmw_request_id_t> reset_header_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, wheel_pub_, truth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
};
}  // namespace capstone_webots_bridge

int main(int argc, char **argv) {
  wb_robot_init();
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto bridge = std::make_shared<capstone_webots_bridge::WebotsBridge>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(bridge);
    while (rclcpp::ok()) {
      executor.spin_some();
      bridge->before_step();
      if (wb_robot_step(bridge->step_ms()) == -1) break;
      bridge->after_step();
    }
    bridge->stop_motors();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "Webots bridge failed: %s\n", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  wb_robot_cleanup();
  return result;
}
