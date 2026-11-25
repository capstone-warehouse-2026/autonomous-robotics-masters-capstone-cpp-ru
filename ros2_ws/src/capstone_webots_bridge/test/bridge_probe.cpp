#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Проверка только драйвера в отдельном ROS_DOMAIN_ID без Nav2/safety_monitor.
// Эталонная поза разрешена здесь, поскольку это evaluator, а не управляющий алгоритм.
class BridgeProbe : public rclcpp::Node {
 public:
  BridgeProbe() : Node("bridge_physics_probe", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})) {
    command_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("state/odom", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr value) { odom_ = *value; have_odom_ = true; });
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("evaluation/ground_truth", 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr value) { truth_ = *value; have_truth_ = true; });
    contact_sub_ = create_subscription<std_msgs::msg::Bool>("evaluation/contact", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr value) { collided_ = collided_ || value->data; });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("sensors/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr value) {
          if (phase_ == 0 && value->ranges.size() == 360) {
            front_range_ = value->ranges[180];
            back_range_ = value->ranges[0];
            have_scan_ = true;
          }
        });
    reset_ = create_client<std_srvs::srv::Trigger>("simulation/reset");
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
  }
  int result() const { return result_; }
 private:
  void fail(const char *message) { std::fprintf(stderr, "FAIL: %s\n", message); result_ = 1; }
  void tick() {
    if (result_ >= 0) return;
    if (std::chrono::steady_clock::now() - started_ > std::chrono::seconds(45)) {
      fail("physical bridge probe timeout"); return;
    }
    const double time = now().seconds();
    if (phase_ == 0) {
      if (!have_odom_ || !have_truth_ || !have_scan_ || time == 0) return;
      if (std::abs(front_range_ - 2.5) > .12 || std::abs(back_range_ - 2.9) > .12) {
        std::fprintf(stderr, "front=%f back=%f\n", front_range_, back_range_);
        fail("real LiDAR orientation/range does not match world"); return;
      }
      initial_truth_x_ = truth_.pose.pose.position.x;
      start_time_ = time;
      phase_ = 1;
    }
    if (phase_ == 1 && time - start_time_ < 4.0) {
      geometry_msgs::msg::TwistStamped command;
      command.header.stamp = now();
      command.header.frame_id = "base_link";
      command.twist.linear.x = .2;
      command_->publish(command);
      return;
    }
    if (phase_ == 1 && time - start_time_ >= 5.0) {
      const double physical = truth_.pose.pose.position.x - initial_truth_x_;
      const double encoder = odom_.pose.pose.position.x;
      std::printf("physical_distance=%f encoder_distance=%f contact=%d physical_v=%f encoder_v=%f yaw_rate=%f\n",
                  physical, encoder, collided_, truth_.twist.twist.linear.x,
                  odom_.twist.twist.linear.x, truth_.twist.twist.angular.z);
      if (physical < .5 || physical > 1.2 || std::abs(physical - encoder) > .12 || collided_) {
        fail("wheel commands did not produce consistent physical displacement"); return;
      }
      if (std::abs(truth_.twist.twist.linear.x) > .03 || std::abs(odom_.twist.twist.linear.x) > .03) {
        fail("driver watchdog did not stop the physical robot"); return;
      }
      if (!reset_->service_is_ready()) { fail("reset service unavailable"); return; }
      phase_ = 2;
      reset_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
          [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            if (!response.get()->success) fail("reset rejected");
            else reset_acknowledged_ = true;
          });
      return;
    }
    if (phase_ == 2 && reset_acknowledged_ && time < 2 && time > .2) {
      if (std::abs(truth_.pose.pose.position.x + 3) > .05 || std::abs(odom_.pose.pose.position.x) > .05) {
        fail("physical reset or encoder reset failed"); return;
      }
      std::puts("PASS: real LiDAR, physical wheel movement, encoder odometry, watchdog and reset");
      result_ = 0;
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry odom_, truth_;
  bool have_odom_{false}, have_truth_{false}, have_scan_{false}, collided_{false}, reset_acknowledged_{false};
  int phase_{0}, result_{-1};
  double front_range_{0}, back_range_{0}, initial_truth_x_{0}, start_time_{0};
  std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto probe = std::make_shared<BridgeProbe>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  while (rclcpp::ok() && probe->result() < 0) executor.spin_once(std::chrono::milliseconds(20));
  const int result = probe->result();
  rclcpp::shutdown();
  return result < 0 ? 1 : result;
}
