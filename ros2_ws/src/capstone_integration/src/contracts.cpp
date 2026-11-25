#include "capstone_integration/contracts.hpp"

namespace capstone_integration {

const std::vector<Contract> &contracts() {
  // Rates are the nominal values of docs/INTERFACES.md; the audit warns outside +-20%.
  static const std::vector<Contract> table = {
      {"/clock", "rosgraph_msgs/msg/Clock", 0.0, 0.0, "", 1, 1, false},
      {"/sensors/wheel_odom", "nav_msgs/msg/Odometry", 50.0, 50.0, "odom", 1, 1, false},
      {"/sensors/imu", "sensor_msgs/msg/Imu", 50.0, 50.0, "imu_link", 1, 1, false},
      {"/sensors/scan", "sensor_msgs/msg/LaserScan", 10.0, 10.0, "base_link", 1, 1, false},
      {"/sensors/rgb", "sensor_msgs/msg/Image", 10.0, 10.0, "camera_optical_frame", 1, 1, false},
      {"/sensors/depth", "sensor_msgs/msg/Image", 10.0, 10.0, "camera_optical_frame", 1, 1, false},
      {"/sensors/camera_info", "sensor_msgs/msg/CameraInfo", 10.0, 10.0,
       "camera_optical_frame", 1, 1, false},
      {"/state/odom", "nav_msgs/msg/Odometry", 50.0, 50.0, "odom", 1, 1, false},
      {"/localization/pose", "geometry_msgs/msg/PoseWithCovarianceStamped", 10.0, 10.0, "map", 1,
       1, false},
      {"/map", "nav_msgs/msg/OccupancyGrid", 0.0, 0.0, "map", 1, 1, true},
      {"/perception/obstacles", "nav_msgs/msg/OccupancyGrid", 10.0, 10.0, "odom", 1, 1, false},
      {"/control/cmd_vel_raw", "geometry_msgs/msg/TwistStamped", 20.0, 20.0, "base_link", 1, 1,
       false},
      {"/cmd_vel", "geometry_msgs/msg/TwistStamped", 20.0, 20.0, "base_link", 1, 1, false},
      {"/mission/status", "diagnostic_msgs/msg/DiagnosticArray", 1.0, 1.0, "", 1, 1, false},
      {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", 0.5, 20.0, "", 0, 99, false, false},
      {"/evaluation/ground_truth", "nav_msgs/msg/Odometry", 0.0, 0.0, "map", 1, 1, true, false},
  };
  return table;
}

bool is_allowed_extra_topic(const std::string &name) {
  // Action plumbing appears as topics (/action/status, /action/feedback) and lifecycle nodes
  // publish /<node>/transition_event; neither is part of the data contract.
  if (name.find("/_action/") != std::string::npos) {
    return true;
  }
  static const std::string transition_suffix = "/transition_event";
  if (name.size() > transition_suffix.size() &&
      name.compare(name.size() - transition_suffix.size(), transition_suffix.size(),
                   transition_suffix) == 0) {
    return true;
  }
  static const std::vector<std::string> exact = {
      "/rosout", "/parameter_events", "/tf", "/tf_static", "/remove_urdf_robot",
      "/Ros2Supervisor/remove_node",
  };
  for (const auto &allowed : exact) {
    if (name == allowed) {
      return true;
    }
  }
  static const std::vector<std::string> prefixes = {
      "/tiago_base/", "/Ros2Supervisor/",
  };
  for (const auto &prefix : prefixes) {
    if (name.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

}  // namespace capstone_integration
