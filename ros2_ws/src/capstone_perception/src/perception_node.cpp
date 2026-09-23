#include "capstone_perception/perception_node.hpp"

#include <stdexcept>

namespace capstone_perception {

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("perception", options),
      method_(declare_parameter<std::string>("method", "classical")) {
  if (method_ != "classical" && method_ != "learned") {
    throw std::invalid_argument("method must be 'classical' or 'learned', got '" + method_ + "'");
  }
  RCLCPP_INFO(get_logger(), "skeleton started with method=%s; nothing is implemented yet",
              method_.c_str());
}

}  // namespace capstone_perception
