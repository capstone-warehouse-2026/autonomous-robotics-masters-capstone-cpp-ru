#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "capstone_perception/grid_builder.hpp"
#include "capstone_perception/pixel_classifier.hpp"

namespace capstone_perception {

// Creates the classifier of a method: "classical" (method A) or "learned" (method B).
std::unique_ptr<PixelClassifier> make_classifier(const std::string &method, rclcpp::Node &node);

// G2 perception node, shared by both methods: synchronizes /sensors/rgb, /sensors/depth and
// /sensors/camera_info, asks the selected PixelClassifier for pixel classes, projects them with
// GridBuilder and publishes /perception/obstacles in target_frame; reports health on /diagnostics.
// The learned method falls back to the classical one when it throws or returns invalid labels.
class PerceptionNode : public rclcpp::Node {
 public:
  explicit PerceptionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::string &method() const { return method_; }

 private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ExactTime<Image, Image, CameraInfo>;

  void on_frame(const Image::ConstSharedPtr &rgb, const Image::ConstSharedPtr &depth,
                const CameraInfo::ConstSharedPtr &info);
  cv::Mat classify(const RgbdFrame &frame);
  void publish_grid(const LocalGrid &grid, const rclcpp::Time &stamp);
  void publish_diagnostics();
  void fail(const std::string &reason);

  std::string method_;
  std::string target_frame_;
  std::string base_frame_;
  double max_input_age_ = 0.3;
  double tf_timeout_ = 0.05;
  std::unique_ptr<PixelClassifier> classifier_;
  std::unique_ptr<PixelClassifier> fallback_;  // classical, only for the learned method
  std::optional<GridBuilder> grid_builder_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<Image> rgb_subscriber_;
  message_filters::Subscriber<Image> depth_subscriber_;
  message_filters::Subscriber<CameraInfo> info_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Health reported on /diagnostics.
  std::optional<rclcpp::Time> last_frame_stamp_;
  std::size_t frames_ = 0;
  std::size_t fallbacks_ = 0;
  std::size_t failures_ = 0;
  double last_processing_ms_ = 0.0;
  std::string last_error_;
};

}  // namespace capstone_perception
