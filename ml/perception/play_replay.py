"""Publish a grid replay as ROS RGB-D, CameraInfo, clock and camera transforms."""

import argparse
from pathlib import Path
import struct
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


HEADER = struct.Struct('<II4d32d')


def read_exact(stream, size):
    data = stream.read(size)
    if len(data) != size:
        raise ValueError('truncated RGB-D replay')
    return data


def transform(parent, child, matrix, stamp):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id, msg.header.stamp = parent, child, stamp
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = matrix[:3, 3].tolist()
    # Select the largest quaternion component to avoid division near a half turn.
    r = matrix[:3, :3]
    q_outer = np.zeros((4, 4))
    q_outer[:3, :3] = r + r.T + np.eye(3) * (1 - np.trace(r))
    q_outer[3, 3] = 1 + np.trace(r)
    q_outer[:3, 3] = q_outer[3, :3] = [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]
    column = q_outer[:, np.argmax(q_outer.diagonal())]
    q = (column / np.linalg.norm(column)).tolist()
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
    return msg


def image(header, width, height, encoding, bytes_per_pixel, data):
    return Image(header=header, width=width, height=height, encoding=encoding,
                 step=width * bytes_per_pixel, data=data)


def play(node, path, limit):
    rgb_pub = node.create_publisher(Image, '/sensors/rgb', qos_profile_sensor_data)
    depth_pub = node.create_publisher(Image, '/sensors/depth', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/sensors/camera_info', qos_profile_sensor_data)
    clock_pub = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    static_tf = StaticTransformBroadcaster(node)
    tf = TransformBroadcaster(node)
    frame_index = 0
    with path.open('rb') as stream:
        if read_exact(stream, 8) != b'G2RGBD01':
            raise ValueError('unsupported RGB-D replay format')
        # Let subscribers and the TF listener discover publishers before the first frame.
        time.sleep(1.0)
        deadline = time.monotonic()
        initial_mount = None
        while rclpy.ok() and (limit is None or frame_index < limit):
            raw = stream.read(HEADER.size)
            if not raw:
                break
            height, width, fx, fy, cx, cy, *poses = HEADER.unpack(raw)
            if (height, width) != (480, 640):
                raise ValueError('expected 640x480 RGB-D frame')
            pixels = height * width
            rgb = read_exact(stream, pixels * 3)
            depth = read_exact(stream, pixels * 4)
            read_exact(stream, pixels * 5)  # clean depth and labels are evaluator inputs
            mount, pose = np.asarray(poses).reshape(2, 4, 4)
            stamp = Time(sec=1 + frame_index // 10, nanosec=(frame_index % 10) * 100_000_000)
            if initial_mount is None:
                initial_mount = mount
                static_tf.sendTransform(transform('base_link', 'camera_optical_frame', mount, stamp))
            elif not np.array_equal(mount, initial_mount):
                raise ValueError('camera mount changed in replay')
            time.sleep(max(0.0, deadline - time.monotonic()))
            clock_pub.publish(Clock(clock=stamp))
            tf.sendTransform(transform('odom', 'base_link', pose, stamp))
            # Allow clock and TF subscribers to handle their messages before the synchronized triplet.
            time.sleep(0.02)
            info = CameraInfo(width=width, height=height, k=[fx, 0., cx, 0., fy, cy, 0., 0., 1.])
            info.header.stamp = stamp
            info.header.frame_id = 'camera_optical_frame'
            rgb_pub.publish(image(info.header, width, height, 'rgb8', 3, rgb))
            depth_pub.publish(image(info.header, width, height, '32FC1', 4, depth))
            info_pub.publish(info)
            frame_index += 1
            deadline += 0.1
    node.get_logger().info(f'Published {frame_index} RGB-D frames at 10 Hz')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay', type=Path)
    parser.add_argument('--limit', type=int)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('g2_replay')
    try:
        play(node, args.replay, args.limit)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
