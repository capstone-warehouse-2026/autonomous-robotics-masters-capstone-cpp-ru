#include "capstone_sim_rgbd/rgbd_camera.hpp"

#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

namespace capstone_sim_rgbd {

namespace {

using Parameters = std::unordered_map<std::string, std::string>;

std::string parameter(const Parameters &parameters, const std::string &name, const std::string &fallback) {
  const auto it = parameters.find(name);
  return it == parameters.end() ? fallback : it->second;
}

std::array<double, 3> vector3(const Parameters &parameters, const std::string &name, const std::string &fallback) {
  std::istringstream stream(parameter(parameters, name, fallback));
  std::array<double, 3> value{};
  if (!(stream >> value[0] >> value[1] >> value[2])) {
    throw std::runtime_error("RgbdCamera: URDF parameter <" + name + "> must hold three numbers");
  }
  return value;
}

WbDeviceTag device(const std::string &name) {
  const WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (tag == 0) {
    throw std::runtime_error("RgbdCamera: device '" + name + "' not found in the Webots world");
  }
  return tag;
}

geometry_msgs::msg::TransformStamped transform(const std::string &parent, const std::string &child,
                                               const tf2::Vector3 &translation, const tf2::Quaternion &rotation) {
  geometry_msgs::msg::TransformStamped message;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  message.transform.translation.x = translation.x();
  message.transform.translation.y = translation.y();
  message.transform.translation.z = translation.z();
  message.transform.rotation.x = rotation.x();
  message.transform.rotation.y = rotation.y();
  message.transform.rotation.z = rotation.z();
  message.transform.rotation.w = rotation.w();
  return message;
}

}  // namespace

void RgbdCamera::init(webots_ros2_driver::WebotsNode *node, Parameters &parameters) {
  node_ = node;
  color_ = device(parameter(parameters, "colorCamera", "Astra rgb"));
  depth_ = device(parameter(parameters, "depthCamera", "Astra depth"));

  const double rate = std::stod(parameter(parameters, "updateRate", "10"));
  const int period_ms = static_cast<int>(std::lround(1000.0 / rate));
  const int basic_step = static_cast<int>(wb_robot_get_basic_time_step());
  if (rate <= 0.0 || period_ms % basic_step != 0) {
    throw std::runtime_error("RgbdCamera: 1000 / updateRate must be a multiple of basicTimeStep");
  }
  period_ = period_ms / 1000.0;
  wb_camera_enable(color_, period_ms);
  wb_range_finder_enable(depth_, period_ms);
  next_publish_ = wb_robot_get_time() + period_;

  const int width = wb_camera_get_width(color_);
  const int height = wb_camera_get_height(color_);
  if (width != wb_range_finder_get_width(depth_) || height != wb_range_finder_get_height(depth_) ||
      std::abs(wb_camera_get_fov(color_) - wb_range_finder_get_fov(depth_)) > 1e-6) {
    throw std::runtime_error("RgbdCamera: color and depth cameras must have the same size and field of view");
  }
  intrinsics_ = webots_intrinsics(width, height, wb_camera_get_fov(color_));
  registered_.resize(static_cast<std::size_t>(width) * height);

  // Device positions inside the camera model (FLU, m); the defaults are those of Astra.proto (R2025a).
  const auto color_position = vector3(parameters, "colorPosition", "0.027 0.011 0.034");
  const auto depth_position = vector3(parameters, "depthPosition", "0.027 0.037 0.034");
  // FLU (x forward, y left, z up) -> optical (x right, y down, z forward).
  depth_offset_ = {-(depth_position[1] - color_position[1]), -(depth_position[2] - color_position[2]),
                   depth_position[0] - color_position[0]};

  const std::string frame = parameter(parameters, "opticalFrame", "camera_optical_frame");
  rgb_message_.header.frame_id = frame;
  rgb_message_.height = height;
  rgb_message_.width = width;
  rgb_message_.encoding = sensor_msgs::image_encodings::RGB8;
  rgb_message_.step = 3 * width;
  rgb_message_.data.resize(rgb_message_.step * height);
  depth_message_.header.frame_id = frame;
  depth_message_.height = height;
  depth_message_.width = width;
  depth_message_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_message_.step = sizeof(float) * width;
  depth_message_.data.resize(depth_message_.step * height);
  info_message_.header.frame_id = frame;
  info_message_.height = height;
  info_message_.width = width;
  info_message_.distortion_model = "plumb_bob";
  info_message_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info_message_.k = {intrinsics_.fx, 0.0, intrinsics_.cx, 0.0, intrinsics_.fy, intrinsics_.cy, 0.0, 0.0, 1.0};
  info_message_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info_message_.p = {intrinsics_.fx, 0.0, intrinsics_.cx, 0.0, 0.0, intrinsics_.fy, intrinsics_.cy, 0.0,
                     0.0, 0.0, 1.0, 0.0};

  rgb_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/sensors/rgb", rclcpp::SensorDataQoS());
  depth_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/sensors/depth", rclcpp::SensorDataQoS());
  info_publisher_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("/sensors/camera_info", rclcpp::SensorDataQoS());

  static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
  publish_static_tf(vector3(parameters, "mountTranslation", "0 0 0"), vector3(parameters, "mountRotation", "0 0 0"),
                    color_position);
  RCLCPP_INFO(node_->get_logger(), "RGB-D camera ready: %dx%d, %.0f Hz, fx %.1f, depth offset %.3f m", width,
              height, rate, intrinsics_.fx, depth_offset_[0]);
}

void RgbdCamera::publish_static_tf(const std::array<double, 3> &mount_xyz, const std::array<double, 3> &mount_rpy,
                                   const std::array<double, 3> &color_position) {
  // camera_link is the color camera: the camera model is mounted at mount_xyz with roll/pitch/yaw mount_rpy
  // in base_link, and the color camera sits at color_position inside it without its own rotation.
  tf2::Quaternion mount;
  mount.setRPY(mount_rpy[0], mount_rpy[1], mount_rpy[2]);
  const tf2::Vector3 camera_origin = tf2::Vector3(mount_xyz[0], mount_xyz[1], mount_xyz[2]) +
      tf2::Matrix3x3(mount) * tf2::Vector3(color_position[0], color_position[1], color_position[2]);
  tf2::Quaternion optical;
  optical.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
  auto base_to_camera = transform("base_link", "camera_link", camera_origin, mount);
  auto camera_to_optical = transform("camera_link", info_message_.header.frame_id, tf2::Vector3(0, 0, 0), optical);
  base_to_camera.header.stamp = camera_to_optical.header.stamp = node_->get_clock()->now();
  static_tf_->sendTransform({base_to_camera, camera_to_optical});
}

void RgbdCamera::step() {
  const double now = wb_robot_get_time();
  if (now + 1e-9 < next_publish_) {
    return;
  }
  next_publish_ += period_;
  const auto *bgra = wb_camera_get_image(color_);
  const float *range = wb_range_finder_get_range_image(depth_);
  if (bgra == nullptr || range == nullptr) {
    return;
  }
  const rclcpp::Time stamp(static_cast<int64_t>(std::llround(now * 1e9)), RCL_ROS_TIME);
  rgb_message_.header.stamp = depth_message_.header.stamp = info_message_.header.stamp = stamp;

  bgra_to_rgb(bgra, registered_.size(), rgb_message_.data.data());
  register_depth(range, intrinsics_, depth_offset_, registered_.data());
  std::memcpy(depth_message_.data.data(), registered_.data(), depth_message_.data.size());

  rgb_publisher_->publish(rgb_message_);
  depth_publisher_->publish(depth_message_);
  info_publisher_->publish(info_message_);
}

}  // namespace capstone_sim_rgbd

PLUGINLIB_EXPORT_CLASS(capstone_sim_rgbd::RgbdCamera, webots_ros2_driver::PluginInterface)
