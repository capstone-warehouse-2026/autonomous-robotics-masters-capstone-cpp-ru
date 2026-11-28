#include "capstone_localization/localization_node.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>

#include "capstone_localization/localization.hpp"
namespace capstone_localization {
namespace {
double yaw(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
geometry_msgs::msg::Quaternion quaternion(double a) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(a / 2);
    q.w = std::cos(a / 2);
    return q;
}
void pose_msg(const Filter &f, geometry_msgs::msg::PoseWithCovariance &p) {
    p.pose.position.x = f.pose.x();
    p.pose.position.y = f.pose.y();
    p.pose.orientation = quaternion(f.pose.z());
    p.covariance.fill(0);
    p.covariance[14] = p.covariance[21] = p.covariance[28] = 1e6;
    const int idx[3] = {0, 1, 5};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            p.covariance[idx[i] * 6 + idx[j]] = f.covariance(i, j);
}
}  // namespace
struct LocalizationNode::Impl {
    LocalizationNode &node;
    Localization filter;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    tf2_ros::TransformBroadcaster broadcaster;
    double imu_stamp = -1, imu_rate = 0, imu_timeout;
    bool imu_valid = false;
    bool publish_map_tf;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
    static LocalizationConfig config(LocalizationNode &n) {
        LocalizationConfig c;
        c.initial =
            Pose(n.declare_parameter("initial_x", 0.0), n.declare_parameter("initial_y", 0.0),
                 n.declare_parameter("initial_yaw", 0.0));
        c.wheel_timeout = n.declare_parameter("wheel_timeout", 0.3);
        return c;
    }
    explicit Impl(LocalizationNode &n)
        : node(n),
          filter(config(n), n.method() == "learned",
                 n.declare_parameter<std::string>("model_path", "")),
          buffer(n.get_clock()),
          listener(buffer),
          broadcaster(n),
          imu_timeout(n.declare_parameter("imu_timeout", 0.1)) {
        publish_map_tf = n.declare_parameter("publish_map_tf", true);
        if (!std::isfinite(imu_timeout) || imu_timeout <= 0)
            throw std::invalid_argument("imu_timeout must be positive");
        odom = n.create_publisher<nav_msgs::msg::Odometry>("/state/odom", 10);
        pose = n.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", 10);
        diagnostics = n.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        imu = n.create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                if (m->header.frame_id != "imu_link" && m->header.frame_id != "base_link") {
                    imu_valid = false;
                    return;
                }
                imu_stamp = rclcpp::Time(m->header.stamp).seconds();
                imu_rate = m->angular_velocity.z;
                imu_valid = std::isfinite(imu_rate) && m->angular_velocity_covariance[0] >= 0;
            });
        wheel = n.create_subscription<nav_msgs::msg::Odometry>(
            "/sensors/wheel_odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                if (m->child_frame_id != "base_link")
                    return;
                double t = rclcpp::Time(m->header.stamp).seconds();
                if (filter.estimate() && t < filter.estimate()->stamp - 0.5) {
                    imu_valid = false;
                }
                std::optional<double> rate;
                if (imu_valid && t >= imu_stamp && t - imu_stamp <= imu_timeout)
                    rate = imu_rate;
                if (filter.wheel(t, m->twist.twist.linear.x, m->twist.twist.angular.z, rate))
                    publish(m->header.stamp);
            });
        map = n.create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
                if (m->header.frame_id != "map") {
                    report("map_frame_must_be_map", false);
                    return;
                }
                try {
                    filter.set_map(m->info.width, m->info.height, m->info.resolution,
                                   Pose(m->info.origin.position.x, m->info.origin.position.y,
                                        yaw(m->info.origin.orientation)),
                                   m->data);
                } catch (const std::exception &e) {
                    report(e.what(), false);
                }
            });
        scan = n.create_subscription<sensor_msgs::msg::LaserScan>(
            "/sensors/scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
                if (!std::isfinite(m->angle_increment) || !std::isfinite(m->angle_min) ||
                    m->angle_increment == 0) {
                    report("invalid_scan", false);
                    return;
                }
                Pose extrinsic = Pose::Zero();
                if (m->header.frame_id != "base_link")
                    try {
                        auto tf = buffer.lookupTransform("base_link", m->header.frame_id,
                                                         rclcpp::Time(m->header.stamp));
                        extrinsic = Pose(tf.transform.translation.x, tf.transform.translation.y,
                                         yaw(tf.transform.rotation));
                    } catch (const tf2::TransformException &) {
                        report("missing_scan_transform", false);
                        return;
                    }
                Points points;
                points.reserve(m->ranges.size());
                for (size_t i = 0; i < m->ranges.size(); ++i) {
                    double r = m->ranges[i], a = m->angle_min + i * m->angle_increment;
                    if (std::isfinite(r) && r >= m->range_min && r <= m->range_max)
                        points.push_back(
                            compose(extrinsic, Pose(r * std::cos(a), r * std::sin(a), 0))
                                .head<2>());
                }
                auto update = filter.scan(rclcpp::Time(m->header.stamp).seconds(), points);
                report(update.reason, update.accepted && !update.fallback);
                // TF is emitted only at wheel timestamps; the next wheel tick publishes
                // corrections.
            });
        if (!filter.model_error().empty())
            RCLCPP_WARN(n.get_logger(), "Learned model unavailable: %s; fixed covariance fallback",
                        filter.model_error().c_str());
    }
    void report(const std::string &reason, bool ok) {
        diagnostic_msgs::msg::DiagnosticArray a;
        a.header.stamp = node.now();
        diagnostic_msgs::msg::DiagnosticStatus s;
        s.name = "g1_localization";
        s.hardware_id = "simulation";
        s.level = ok ? 0 : 1;
        s.message = reason;
        a.status.push_back(s);
        diagnostics->publish(a);
    }
    void publish(const builtin_interfaces::msg::Time &stamp) {
        const auto &e = *filter.estimate();
        nav_msgs::msg::Odometry o;
        o.header.stamp = stamp;
        o.header.frame_id = "odom";
        o.child_frame_id = "base_link";
        pose_msg(e.odom, o.pose);
        o.twist.twist.linear.x = e.forward_velocity;
        o.twist.twist.angular.z = e.yaw_rate;
        odom->publish(o);
        geometry_msgs::msg::PoseWithCovarianceStamped p;
        p.header.stamp = stamp;
        p.header.frame_id = "map";
        pose_msg(e.global, p.pose);
        pose->publish(p);
        auto tf = [&](const std::string &parent, const std::string &child, const Pose &v) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp;
            t.header.frame_id = parent;
            t.child_frame_id = child;
            t.transform.translation.x = v.x();
            t.transform.translation.y = v.y();
            t.transform.rotation = quaternion(v.z());
            return t;
        };
        Pose map_odom = compose(e.global.pose, between(e.odom.pose, Pose::Zero()));
        std::vector<geometry_msgs::msg::TransformStamped> transforms{
            tf("odom", "base_link", e.odom.pose)};
        if (publish_map_tf)
            transforms.push_back(tf("map", "odom", map_odom));
        broadcaster.sendTransform(transforms);
    }
};
LocalizationNode::LocalizationNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("localization", options),
      method_(declare_parameter<std::string>("method", "classical")) {
    if (method_ != "classical" && method_ != "learned")
        throw std::invalid_argument("method must be classical or learned");
    impl_ = std::make_unique<Impl>(*this);
}
LocalizationNode::~LocalizationNode() = default;
}  // namespace capstone_localization
