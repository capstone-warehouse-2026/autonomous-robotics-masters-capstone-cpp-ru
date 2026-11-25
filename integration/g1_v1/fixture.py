#!/usr/bin/env python3
"""Synthetic G1 inputs and optional mock outputs, for isolated development only."""
import argparse
import math
import os
import signal
import struct
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

MAP = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CLOCK = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
SENSOR = qos_profile_sensor_data
STATE = QoSProfile(depth=5)
OPTICAL = "camera_optical_frame"
INPUTS = (
    (Clock, "/clock", CLOCK, None),
    (OccupancyGrid, "/map", MAP, "map"),
    (Odometry, "/sensors/wheel_odom", SENSOR, "odom"),
    (Imu, "/sensors/imu", SENSOR, "imu_link"),
    (LaserScan, "/sensors/scan", SENSOR, "lidar_link"),
    (Image, "/sensors/rgb", SENSOR, OPTICAL),
    (Image, "/sensors/depth", SENSOR, OPTICAL),
    (CameraInfo, "/sensors/camera_info", SENSOR, OPTICAL),
)
OUTPUTS = (
    (Odometry, "/state/odom", STATE, "odom"),
    (PoseWithCovarianceStamped, "/localization/pose", STATE, "map"),
)
STATIC = {("base_link", "imu_link"), ("base_link", "lidar_link"),
          ("base_link", "camera_link"), ("camera_link", OPTICAL)}
DYNAMIC = {("map", "odom"), ("odom", "base_link")}
COV = [0.02 if i in (0, 7, 35) else 0.0 for i in range(36)]


def transform(parent, child, stamp, x=0.0):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    if child == OPTICAL:
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    if child == "camera_link":
        t.transform.translation.z = 0.5
    return t


class Fixture(Node):
    def __init__(self, mock):
        super().__init__("g1_v1_fixture")
        self.outputs = mock
        self.pubs = {name: self.create_publisher(msg, name, qos)
                     for msg, name, qos, _ in INPUTS + (OUTPUTS if mock else ())}
        self.static = StaticTransformBroadcaster(self)
        self.dynamic = TransformBroadcaster(self) if mock else None
        self.i = 0
        self.create_timer(0.02, self.tick)

    def publish(self, topic, msg, stamp, frame):
        msg.header.stamp, msg.header.frame_id = stamp, frame
        self.pubs[topic].publish(msg)

    def tick(self):
        self.i += 1
        clock = Clock()
        stamp = clock.clock
        stamp.sec, stamp.nanosec = divmod(1_000_000_000 + self.i * 20_000_000, 1_000_000_000)
        self.pubs["/clock"].publish(clock)
        if self.i == 1:
            self.static.sendTransform([transform(p, c, stamp) for p, c in sorted(STATIC)])
            grid = OccupancyGrid()
            grid.info.map_load_time = stamp
            grid.info.width = grid.info.height = 80
            grid.info.resolution = 0.1
            grid.info.origin.position.x = grid.info.origin.position.y = -4.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [100 if i // 80 in (0, 79) or i % 80 in (0, 79) else 0
                         for i in range(6400)]
            self.publish("/map", grid, stamp, "map")

        x = 0.5 * math.sin(self.i * 0.008)
        wheel = Odometry()
        wheel.child_frame_id = "base_link"
        wheel.pose.pose.position.x, wheel.pose.pose.orientation.w = x, 1.0
        wheel.pose.covariance = COV
        wheel.twist.twist.linear.x = 0.2 * math.cos(self.i * 0.008)
        imu = Imu()
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        imu.angular_velocity_covariance[8] = 0.002
        self.publish("/sensors/imu", imu, stamp, "imu_link")
        self.publish("/sensors/wheel_odom", wheel, stamp, "odom")
        if self.outputs:
            self.publish("/state/odom", wheel, stamp, "odom")
            self.dynamic.sendTransform([transform("map", "odom", stamp),
                                        transform("odom", "base_link", stamp, x)])
        if self.i % 5:
            return
        if self.outputs:
            pose = PoseWithCovarianceStamped()
            pose.pose = wheel.pose
            self.publish("/localization/pose", pose, stamp, "map")
        scan = LaserScan()
        scan.angle_min, scan.angle_increment = -math.pi, math.tau / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = 0.05, 10.0, 0.1
        for i in range(360):
            a = -math.pi + i * math.tau / 360
            c, s = math.cos(a), math.sin(a)
            rx = ((3.9 if c > 0 else -3.9) - x) / c if abs(c) > 1e-9 else math.inf
            ry = 3.9 / abs(s) if abs(s) > 1e-9 else math.inf
            scan.ranges.append(min(rx, ry))
        self.publish("/sensors/scan", scan, stamp, "lidar_link")

        w, h = 64, 48
        for name, enc, pixel in (("rgb", "rgb8", bytes([48, 120, 192])),
                                 ("depth", "32FC1", struct.pack("<f", 2.0))):
            image = Image()
            image.width, image.height, image.encoding = w, h, enc
            image.step, image.data = w * len(pixel), pixel * (w * h)
            self.publish("/sensors/" + name, image, stamp, OPTICAL)
        info = CameraInfo()
        info.width, info.height = w, h
        info.distortion_model, info.d = "plumb_bob", [0.0] * 5
        info.k = [50.0, 0.0, 31.5, 0.0, 50.0, 23.5, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [50.0, 0.0, 31.5, 0.0, 0.0, 50.0, 23.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.publish("/sensors/camera_info", info, stamp, OPTICAL)


def check(node, timeout):
    seen, edges, errors = set(), set(), set()
    expected = {topic for _, topic, _, _ in INPUTS + OUTPUTS}
    clocks = []

    def inspect(msg, topic, frame):
        stamp = msg.clock if topic == "/clock" else msg.header.stamp
        if stamp.sec < 1 or not 0 <= stamp.nanosec < 1_000_000_000:
            errors.add(f"{topic}: bad stamp")
        if frame is not None and msg.header.frame_id != frame:
            errors.add(f"{topic}: bad frame")
        if topic == "/clock":
            clocks.append((stamp.sec, stamp.nanosec))
        if isinstance(msg, OccupancyGrid) and (
                len(msg.data) != msg.info.width * msg.info.height or set(msg.data) != {0, 100}):
            errors.add(f"{topic}: bad map")
        if isinstance(msg, Odometry) and msg.child_frame_id != "base_link":
            errors.add(f"{topic}: bad child")
        if topic in ("/state/odom", "/localization/pose") and not all(
                math.isfinite(v) for v in msg.pose.covariance):
            errors.add(f"{topic}: nonfinite covariance")
        if isinstance(msg, Image):
            encoding, stride = ("rgb8", 3) if topic.endswith("/rgb") else ("32FC1", 4)
            if (msg.encoding != encoding or msg.step != msg.width * stride
                    or len(msg.data) != msg.height * msg.step):
                errors.add(f"{topic}: bad image layout")
        if isinstance(msg, LaserScan) and (len(msg.ranges) != 360 or not all(
                msg.range_min <= r <= msg.range_max for r in msg.ranges)):
            errors.add(f"{topic}: bad ranges")
        seen.add(topic)

    def inspect_tf(msg, allowed):
        for t in msg.transforms:
            edge = (t.header.frame_id, t.child_frame_id)
            if edge not in allowed:
                errors.add(f"unexpected TF {edge}")
            edges.add(edge)

    for msg, topic, qos, frame in INPUTS + OUTPUTS:
        node.create_subscription(msg, topic, lambda m, t=topic, f=frame: inspect(m, t, f), qos)
    node.create_subscription(TFMessage, "/tf", lambda m: inspect_tf(m, DYNAMIC), 10)
    node.create_subscription(TFMessage, "/tf_static", lambda m: inspect_tf(m, STATIC), MAP)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if errors:
            break
        if seen == expected and edges == STATIC | DYNAMIC and len(clocks) >= 5:
            if any(b <= a for a, b in zip(clocks, clocks[1:])):
                errors.add("/clock not advancing")
                break
            exclusive = ["/clock", "/map", "/tf", "/tf_static", "/state/odom", "/localization/pose"]
            if all(node.count_publishers(topic) == 1 for topic in exclusive):
                print(f"PASS G1 v1: {len(seen)} topics, {len(edges)} TF edges, advancing clock")
                return 0
    print(f"FAIL G1 v1: missing topics={sorted(expected-seen)}, "
          f"missing TF={sorted((STATIC | DYNAMIC)-edges)}, clock samples={len(clocks)}, "
          f"errors={sorted(errors)}")
    return 1


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("mock", "sensors", "check"))
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    distro = os.environ.get("ROS_DISTRO")
    if distro == "jazzy":
        local_only = (
            os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST"
            and os.environ.get("ROS_LOCALHOST_ONLY") != "0"
            and not os.environ.get("ROS_STATIC_PEERS")
        )
    elif distro == "humble":
        local_only = os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    else:
        local_only = False
    if not local_only or not domain.isdecimal() or not 170 <= int(domain) <= 224:
        parser.error("source Jazzy or Humble, set localhost discovery and use an unused ROS_DOMAIN_ID in 170..224")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, interrupt)
    node = Node("g1_v1_checker") if args.mode == "check" else Fixture(args.mode == "mock")
    try:
        if args.mode == "check":
            return check(node, args.timeout)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
