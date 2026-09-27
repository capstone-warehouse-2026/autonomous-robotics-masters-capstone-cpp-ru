#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <webots/types.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include "capstone_sim_rgbd/rgbd_image.hpp"

namespace capstone_sim_rgbd {

// Simulator adapter of the RGB-D camera (G2), docs/INTERFACES.md:
// /sensors/rgb (rgb8), /sensors/depth (32FC1, m, registered to the color camera, NaN = no measurement),
// /sensors/camera_info and static TF base_link -> camera_link -> camera_optical_frame.
// Device names, rate and mounting come from the robot URDF (capstone_sim/resource/tiago_base.urdf).
class RgbdCamera : public webots_ros2_driver::PluginInterface {
 public:
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
  void step() override;

 private:
  void publish_static_tf(const std::array<double, 3> &mount_xyz, const std::array<double, 3> &mount_rpy,
                         const std::array<double, 3> &color_position);

  webots_ros2_driver::WebotsNode *node_ = nullptr;
  WbDeviceTag color_ = 0;
  WbDeviceTag depth_ = 0;
  Intrinsics intrinsics_;
  std::array<double, 3> depth_offset_{};  // depth camera origin in the color optical frame
  double period_ = 0.1;                    // s
  double next_publish_ = 0.0;              // simulation time, s
  std::vector<float> registered_;
  sensor_msgs::msg::Image rgb_message_;
  sensor_msgs::msg::Image depth_message_;
  sensor_msgs::msg::CameraInfo info_message_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

}  // namespace capstone_sim_rgbd
