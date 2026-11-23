#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "capstone_integration/audit_core.hpp"
#include "capstone_integration/contracts.hpp"

namespace capstone_integration {

// Read-only audit of the live ROS graph against docs/INTERFACES.md. It never publishes commands:
// it samples the contract, then prints a table, writes JSON and publishes /diagnostics.
class GraphAuditNode : public rclcpp::Node {
 public:
  GraphAuditNode();

  bool finished() const { return finished_; }
  int result() const { return exit_code_; }

 private:
  enum class Phase { Settle, Sampling };

  void tick();
  void begin_sampling();
  void finish();
  void count(const std::string &topic);
  void record_frame(const std::string &topic, const std::string &frame);
  void on_clock(const rosgraph_msgs::msg::Clock &message);

  template <typename MessageT>
  void subscribe_topic(const std::string &topic, const rclcpp::QoS &qos) {
    auto subscription = create_subscription<MessageT>(
        topic, qos, [this, topic](const std::shared_ptr<const MessageT> message) {
          count(topic);
          record_frame(topic, message->header.frame_id);
        });
    subscriptions_.push_back(subscription);
  }

  double settle_s_{2.0};
  double window_s_{5.0};
  std::string fail_on_{"error"};
  std::string json_path_;
  bool measured_run_{false};
  bool publish_diagnostics_{true};

  bool sampling_{false};
  bool finished_{false};
  int exit_code_{0};
  Phase phase_{Phase::Settle};
  std::chrono::steady_clock::time_point phase_start_{};

  std::map<std::string, int> counters_;
  std::map<std::string, std::string> frames_;

  int clock_count_{0};
  double clock_first_s_{0.0};
  double clock_last_s_{-1.0};
  bool clock_monotonic_{true};

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace capstone_integration
