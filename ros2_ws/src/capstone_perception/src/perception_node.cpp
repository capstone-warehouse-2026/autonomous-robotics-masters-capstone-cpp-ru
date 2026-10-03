#include "capstone_perception/perception_node.hpp"

#include <stdexcept>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "capstone_perception/cnn_classifier.hpp"
#include "capstone_perception/geometric_classifier.hpp"

namespace capstone_perception {

std::unique_ptr<PixelClassifier> make_classifier(const std::string &method, rclcpp::Node &node) {
  if (method == "classical") {
    return std::make_unique<GeometricClassifier>(node);
  }
  if (method == "learned") {
    return std::make_unique<CnnClassifier>(node);
  }
  throw std::invalid_argument("method must be 'classical' or 'learned', got '" + method + "'");
}

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("perception", options),
      method_(declare_parameter<std::string>("method", "classical")),
      target_frame_(declare_parameter<std::string>("target_frame", "odom")),
      base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
      max_input_age_(declare_parameter<double>("max_input_age", 0.3)),
      tf_timeout_(declare_parameter<double>("tf_timeout", 0.05)) {
  classifier_ = make_classifier(method_, *this);
  if (method_ == "learned") {
    fallback_ = make_classifier("classical", *this);
  }
  grid_builder_.emplace(declare_parameter<double>("grid.resolution", 0.05), declare_parameter<double>("grid.size", 10.0),
                        static_cast<int>(declare_parameter<int>("grid.pixel_stride", 2)),
                        static_cast<int>(declare_parameter<int>("grid.min_free_points", 1)),
                        static_cast<int>(declare_parameter<int>("grid.min_obstacle_points", 1)));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Sensor data QoS (best effort) as published by the camera, docs/INTERFACES.md.
  const auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  rgb_subscriber_.subscribe(this, "/sensors/rgb", sensor_qos);
  depth_subscriber_.subscribe(this, "/sensors/depth", sensor_qos);
  info_subscriber_.subscribe(this, "/sensors/camera_info", sensor_qos);
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), rgb_subscriber_, depth_subscriber_, info_subscriber_);
  synchronizer_->registerCallback(&PerceptionNode::on_frame, this);

  grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/perception/obstacles", rclcpp::QoS(1).reliable().durability_volatile());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_diagnostics(); });

  RCLCPP_INFO(get_logger(), "perception started: method=%s, grid in %s", method_.c_str(), target_frame_.c_str());
}

void PerceptionNode::on_frame(const Image::ConstSharedPtr &rgb, const Image::ConstSharedPtr &depth,
                              const CameraInfo::ConstSharedPtr &info) {
  const auto started = std::chrono::steady_clock::now();
  const rclcpp::Time stamp(depth->header.stamp, RCL_ROS_TIME);
  if (depth->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
    fail("/sensors/depth must be 32FC1, got " + depth->encoding);
    return;
  }
  if (info->k[0] <= 0.0 || info->k[4] <= 0.0 || depth->width != info->width || depth->height != info->height ||
      rgb->width != depth->width || rgb->height != depth->height) {
    fail("rgb, depth and camera_info sizes or intrinsics do not match");
    return;
  }

  RgbdFrame frame;
  frame.stamp = stamp;
  frame.camera = {info->k[0], info->k[4], info->k[2], info->k[5], static_cast<int>(info->width),
                  static_cast<int>(info->height)};
  Eigen::Isometry3d target_from_camera;
  Eigen::Isometry3d target_from_base;
  try {
    frame.rgb = cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::RGB8)->image;
    frame.depth = cv_bridge::toCvShare(depth)->image;
    const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_);
    frame.base_from_camera =
        tf2::transformToEigen(tf_buffer_->lookupTransform(base_frame_, depth->header.frame_id, stamp, timeout));
    target_from_camera =
        tf2::transformToEigen(tf_buffer_->lookupTransform(target_frame_, depth->header.frame_id, stamp, timeout));
    target_from_base = tf2::transformToEigen(tf_buffer_->lookupTransform(target_frame_, base_frame_, stamp, timeout));
  } catch (const std::exception &error) {
    fail(std::string("input or TF not available: ") + error.what());
    return;
  }

  cv::Mat labels;
  try {
    labels = classify(frame);
  } catch (const std::exception &error) {
    fail(error.what());
    return;
  }
  const LocalGrid grid = grid_builder_->build(labels, frame, target_from_camera, target_from_base.translation().x(),
                                              target_from_base.translation().y());
  publish_grid(grid, stamp);
  last_frame_stamp_ = stamp;
  ++frames_;
  last_processing_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
}

cv::Mat PerceptionNode::classify(const RgbdFrame &frame) {
  std::string problem;
  try {
    cv::Mat labels = classifier_->classify(frame);
    problem = validate_labels(labels, frame);
    if (problem.empty()) {
      return labels;
    }
  } catch (const std::exception &error) {
    problem = error.what();
  }
  if (!fallback_) {
    throw std::runtime_error(classifier_->name() + ": " + problem);
  }
  // Learned method failed on this frame: classical fallback, counted and reported (docs/INTERFACES.md).
  ++fallbacks_;
  last_error_ = classifier_->name() + ": " + problem;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s; using the classical fallback", last_error_.c_str());
  cv::Mat labels = fallback_->classify(frame);
  const std::string fallback_problem = validate_labels(labels, frame);
  if (!fallback_problem.empty()) {
    throw std::runtime_error(fallback_->name() + ": " + fallback_problem);
  }
  return labels;
}

void PerceptionNode::publish_grid(const LocalGrid &grid, const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = target_frame_;
  message.info.map_load_time = stamp;
  message.info.resolution = static_cast<float>(grid.resolution);
  message.info.width = grid.width;
  message.info.height = grid.height;
  message.info.origin.position.x = grid.origin_x;
  message.info.origin.position.y = grid.origin_y;
  message.info.origin.orientation.w = 1.0;
  message.data = grid.data;
  grid_publisher_->publish(message);
}

void PerceptionNode::fail(const std::string &reason) {
  // No grid for this frame: consumers must not drive on a stale map (docs/INTERFACES.md).
  ++failures_;
  last_error_ = reason;
  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "frame dropped: %s", reason.c_str());
}

void PerceptionNode::publish_diagnostics() {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "perception";
  status.hardware_id = "capstone_perception";
  if (!last_frame_stamp_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "waiting for /sensors/rgb, /sensors/depth, /sensors/camera_info and TF";
  } else if ((now() - *last_frame_stamp_).seconds() > max_input_age_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "no fresh grid: input stale or frames dropped";
  } else if (fallbacks_ > 0 && method_ == "learned") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "learned method falls back to classical";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ok";
  }
  const auto value = [](const std::string &key, const std::string &text) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = text;
    return pair;
  };
  status.values = {value("method", method_),
                   value("frames", std::to_string(frames_)),
                   value("fallbacks", std::to_string(fallbacks_)),
                   value("dropped_frames", std::to_string(failures_)),
                   value("processing_ms", std::to_string(last_processing_ms_)),
                   value("last_error", last_error_)};
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace capstone_perception
