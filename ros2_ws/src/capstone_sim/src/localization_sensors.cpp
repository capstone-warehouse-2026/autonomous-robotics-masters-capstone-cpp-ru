#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/lidar.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace capstone_sim {
namespace {

using Parameters = std::unordered_map<std::string, std::string>;
constexpr int motion_period_ms = 20;
constexpr int scan_period_ms = 100;
constexpr int64_t ns_per_ms = 1000000;
constexpr double ns_to_seconds = 1e-9;

const std::string &required(const Parameters &parameters, const std::string &key) {
  const auto value = parameters.find(key);
  if (value == parameters.end()) {
    throw std::runtime_error("LocalizationSensors: missing URDF parameter " + key);
  }
  return value->second;
}

double positive(const Parameters &parameters, const std::string &key) {
  const double value = std::stod(required(parameters, key));
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error("LocalizationSensors: expected a positive value for " + key);
  }
  return value;
}

WbDeviceTag device(const Parameters &parameters, const std::string &key) {
  const auto &name = required(parameters, key);
  const WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (tag == 0) {
    throw std::runtime_error("LocalizationSensors: Webots device not found: " + name);
  }
  return tag;
}

int64_t simulation_time_ns() {
  return std::llround(wb_robot_get_time() * 1e9);
}

bool finite_vector(const double *value, int size) {
  return value != nullptr && std::all_of(value, value + size, [](double v) { return std::isfinite(v); });
}

geometry_msgs::msg::TransformStamped static_transform(const std::string &child) {
  geometry_msgs::msg::TransformStamped message;
  message.header.frame_id = "base_link";
  message.child_frame_id = child;
  message.transform.rotation.w = 1.0;
  return message;
}

}  // namespace

class LocalizationSensors : public webots_ros2_driver::PluginInterface {
 public:
  void init(webots_ros2_driver::WebotsNode *node, Parameters &parameters) override {
    const int basic_step = static_cast<int>(wb_robot_get_basic_time_step());
    if (basic_step <= 0 || motion_period_ms % basic_step != 0 || scan_period_ms % basic_step != 0) {
      throw std::runtime_error("LocalizationSensors: basicTimeStep must divide 20 ms and 100 ms");
    }
    wheel_radius_ = positive(parameters, "wheelRadius");
    wheel_separation_ = positive(parameters, "wheelSeparation");
    left_encoder_ = device(parameters, "leftEncoder");
    right_encoder_ = device(parameters, "rightEncoder");
    accelerometer_ = device(parameters, "accelerometer");
    gyro_ = device(parameters, "gyro");
    lidar_ = device(parameters, "lidar");
    wb_position_sensor_enable(left_encoder_, motion_period_ms);
    wb_position_sensor_enable(right_encoder_, motion_period_ms);
    wb_accelerometer_enable(accelerometer_, motion_period_ms);
    wb_gyro_enable(gyro_, motion_period_ms);
    wb_lidar_enable(lidar_, scan_period_ms);

    const int resolution = wb_lidar_get_horizontal_resolution(lidar_);
    if (resolution < 2 || wb_lidar_get_number_of_layers(lidar_) != 1) {
      throw std::runtime_error("LocalizationSensors: expected a single LiDAR layer with at least two rays");
    }
    const auto fov = wb_lidar_get_fov(lidar_);
    scan_.header.frame_id = "lidar_link";
    scan_.angle_min = -fov / 2.0;
    scan_.angle_max = fov / 2.0;
    scan_.angle_increment = fov / (resolution - 1);
    scan_.scan_time = scan_period_ms / 1000.0;
    scan_.time_increment = 0.0F;  // Webots fixed LiDAR renders a simultaneous scan.
    scan_.range_min = wb_lidar_get_min_range(lidar_);
    scan_.range_max = wb_lidar_get_max_range(lidar_);
    scan_.ranges.resize(resolution);

    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";
    odom_.pose.pose.orientation.w = 1.0;
    odom_.pose.covariance[14] = odom_.pose.covariance[21] = odom_.pose.covariance[28] = 1e6;
    odom_.twist.covariance[0] = 0.02 * 0.02;
    odom_.twist.covariance[35] = 0.03 * 0.03;
    odom_.twist.covariance[7] = odom_.twist.covariance[14] =
        odom_.twist.covariance[21] = odom_.twist.covariance[28] = 1e6;
    imu_.header.frame_id = "imu_link";
    // No absolute yaw is available from a real accelerometer/gyro pair. Webots InertialUnit
    // supplies orientation from simulator state and is intentionally not used by the estimator.
    imu_.orientation_covariance[0] = -1.0;
    for (int axis = 0; axis < 3; ++axis) {
      imu_.angular_velocity_covariance[axis * 4] = 0.01 * 0.01;
      imu_.linear_acceleration_covariance[axis * 4] = 0.1 * 0.1;
    }

    const auto qos = rclcpp::SensorDataQoS();
    wheel_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/sensors/wheel_odom", qos);
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", qos);
    scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>("/sensors/scan", qos);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    auto lidar_tf = static_transform(scan_.header.frame_id);
    auto &position = lidar_tf.transform.translation;
    std::istringstream xyz(required(parameters, "lidarTranslation"));
    if (!(xyz >> position.x >> position.y >> position.z)) {
      throw std::runtime_error("LocalizationSensors: lidarTranslation requires three numbers");
    }
    static_tf_->sendTransform({static_transform(imu_.header.frame_id), lidar_tf});
    reset(simulation_time_ns());
    RCLCPP_INFO(node->get_logger(), "G1 sensors ready: encoders/IMU 50 Hz, 2D LiDAR 10 Hz (%d rays)", resolution);
  }

  void step() override {
    const int64_t now = simulation_time_ns();
    if (now < last_step_) {
      reset(now);
    }
    last_step_ = now;
    const rclcpp::Time stamp(now, RCL_ROS_TIME);
    if (now >= next_motion_) {
      next_motion_ = now + motion_period_ms * ns_per_ms;
      publish_wheels(now, stamp);
      publish_imu(stamp);
    }
    if (now >= next_scan_) {
      next_scan_ = now + scan_period_ms * ns_per_ms;
      const auto *ranges = wb_lidar_get_layer_range_image(lidar_, 0);
      if (ranges != nullptr) {
        // Webots stores +FOV/2 first. ROS consumers expect increasing angles.
        std::reverse_copy(ranges, ranges + scan_.ranges.size(), scan_.ranges.begin());
        scan_.header.stamp = stamp;
        scan_pub_->publish(scan_);
      }
    }
  }

 private:
  struct Encoders {
    int64_t time_ns;
    double left;
    double right;
  };

  void reset(int64_t now) {
    last_step_ = now;
    next_motion_ = now + motion_period_ms * ns_per_ms;
    next_scan_ = now + scan_period_ms * ns_per_ms;
    last_encoders_.reset();
    yaw_ = 0.0;
    odom_.pose.pose = geometry_msgs::msg::Pose();
    odom_.pose.pose.orientation.w = 1.0;
    odom_.pose.covariance[0] = odom_.pose.covariance[7] = 1e-6;
    odom_.pose.covariance[35] = 1e-6;
  }

  void publish_wheels(int64_t now, const rclcpp::Time &stamp) {
    const Encoders encoders{now, wb_position_sensor_get_value(left_encoder_),
                            wb_position_sensor_get_value(right_encoder_)};
    if (!std::isfinite(encoders.left) || !std::isfinite(encoders.right)) {
      return;
    }
    const auto previous = last_encoders_;
    last_encoders_ = encoders;
    if (previous) {
      const double dt = (now - previous->time_ns) * ns_to_seconds;
      const double dl = wheel_radius_ * (encoders.left - previous->left);
      const double dr = wheel_radius_ * (encoders.right - previous->right);
      const double distance = (dl + dr) / 2.0;
      const double turn = (dr - dl) / wheel_separation_;
      const double half_turn = turn / 2.0;
      const double chord = distance * (std::abs(half_turn) < 1e-8 ? 1.0 : std::sin(half_turn) / half_turn);
      odom_.pose.pose.position.x += chord * std::cos(yaw_ + half_turn);
      odom_.pose.pose.position.y += chord * std::sin(yaw_ + half_turn);
      yaw_ = std::atan2(std::sin(yaw_ + turn), std::cos(yaw_ + turn));
      odom_.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
      odom_.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
      odom_.twist.twist.linear.x = distance / dt;
      odom_.twist.twist.angular.z = turn / dt;
      odom_.pose.covariance[0] += odom_.twist.covariance[0] * dt * dt;
      odom_.pose.covariance[7] += odom_.twist.covariance[0] * dt * dt;
      odom_.pose.covariance[35] += odom_.twist.covariance[35] * dt * dt;
    } else {
      odom_.twist.twist = geometry_msgs::msg::Twist();
    }
    odom_.header.stamp = stamp;
    wheel_pub_->publish(odom_);
  }

  void publish_imu(const rclcpp::Time &stamp) {
    const double *accel = wb_accelerometer_get_values(accelerometer_);
    const double *gyro = wb_gyro_get_values(gyro_);
    if (!finite_vector(accel, 3) || !finite_vector(gyro, 3)) {
      return;
    }
    imu_.header.stamp = stamp;
    imu_.linear_acceleration.x = accel[0];
    imu_.linear_acceleration.y = accel[1];
    imu_.linear_acceleration.z = accel[2];
    imu_.angular_velocity.x = gyro[0];
    imu_.angular_velocity.y = gyro[1];
    imu_.angular_velocity.z = gyro[2];
    imu_pub_->publish(imu_);
  }

  WbDeviceTag left_encoder_{}, right_encoder_{}, accelerometer_{}, gyro_{}, lidar_{};
  double wheel_radius_{}, wheel_separation_{}, yaw_{};
  int64_t last_step_{}, next_motion_{}, next_scan_{};
  std::optional<Encoders> last_encoders_;
  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::LaserScan scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

class GroundTruth : public webots_ros2_driver::PluginInterface {
 public:
  void init(webots_ros2_driver::WebotsNode *node, Parameters &) override {
    if (!wb_robot_get_supervisor()) {
      throw std::runtime_error("GroundTruth: TIAGo must have supervisor TRUE in the world");
    }
    robot_ = wb_supervisor_node_get_self();
    if (robot_ == nullptr) {
      throw std::runtime_error("GroundTruth: no Supervisor node for TIAGo");
    }
    publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/evaluation/ground_truth", rclcpp::QoS(10));
    message_.header.frame_id = "map";
    message_.child_frame_id = "base_link";
  }

  void step() override {
    const int64_t now = simulation_time_ns();
    if (!last_ || now < *last_ || now - *last_ >= motion_period_ms * ns_per_ms) {
      last_ = now;
      const double *position = wb_supervisor_node_get_position(robot_);
      const double *rotation = wb_supervisor_node_get_orientation(robot_);
      const double *velocity = wb_supervisor_node_get_velocity(robot_);
      if (!finite_vector(position, 3) || !finite_vector(rotation, 9) || !finite_vector(velocity, 6)) {
        return;
      }
      const tf2::Matrix3x3 basis(rotation[0], rotation[1], rotation[2], rotation[3], rotation[4], rotation[5],
                                rotation[6], rotation[7], rotation[8]);
      tf2::Quaternion q;
      basis.getRotation(q);
      const auto inverse = basis.transpose();
      const auto linear = inverse * tf2::Vector3(velocity[0], velocity[1], velocity[2]);
      const auto angular = inverse * tf2::Vector3(velocity[3], velocity[4], velocity[5]);
      message_.header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
      auto &pose = message_.pose.pose;
      pose.position.x = position[0];
      pose.position.y = position[1];
      pose.position.z = position[2];
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      auto &twist = message_.twist.twist;
      twist.linear.x = linear.x();
      twist.linear.y = linear.y();
      twist.linear.z = linear.z();
      twist.angular.x = angular.x();
      twist.angular.y = angular.y();
      twist.angular.z = angular.z();
      publisher_->publish(message_);
    }
  }

 private:
  WbNodeRef robot_ = nullptr;
  std::optional<int64_t> last_;
  nav_msgs::msg::Odometry message_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

}  // namespace capstone_sim

PLUGINLIB_EXPORT_CLASS(capstone_sim::LocalizationSensors, webots_ros2_driver::PluginInterface)
PLUGINLIB_EXPORT_CLASS(capstone_sim::GroundTruth, webots_ros2_driver::PluginInterface)
