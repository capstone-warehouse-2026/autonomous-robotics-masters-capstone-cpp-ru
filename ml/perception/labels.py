"""Ground-truth pixel labels for method B, computed from clean depth and the exact camera pose.

Offline tool (Python is allowed for data preparation only). The classes are those of
capstone_perception::PixelClass: the network learns to output them from RGB-D.
"""

import numpy as np

FREE = 0
OBSTACLE = 1
UNKNOWN = 255


def pixel_heights(depth, camera, world_from_camera):
    """Height above the floor (world z, floor at z = 0) of every pixel; NaN where the depth is NaN.

    depth: HxW float32, metres along the optical axis, NaN = no measurement.
    camera: dict with fx, fy, cx, cy (ROS CameraInfo convention).
    world_from_camera: 4x4 pose of camera_optical_frame (z forward, x right, y down) in the world.
    """
    height, width = depth.shape
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    z = depth.astype(np.float64)
    x = (u - camera['cx']) * z / camera['fx']
    y = (v - camera['cy']) * z / camera['fy']
    rotation = world_from_camera[:3, :3]
    translation = world_from_camera[:3, 3]
    return rotation[2, 0] * x + rotation[2, 1] * y + rotation[2, 2] * z + translation[2]


def compute_labels(depth, camera, world_from_camera, floor_tolerance=0.02, robot_height=1.0):
    """Pixel classes by height over the floor.

    |h| <= floor_tolerance -> FREE; floor_tolerance < h <= robot_height -> OBSTACLE;
    h < -floor_tolerance (a hole or an edge below the floor) -> OBSTACLE;
    h > robot_height (the robot passes under it) and no depth -> UNKNOWN.
    """
    heights = pixel_heights(depth, camera, world_from_camera)
    labels = np.full(depth.shape, UNKNOWN, dtype=np.uint8)
    valid = np.isfinite(depth)
    labels[valid & (np.abs(heights) <= floor_tolerance)] = FREE
    labels[valid & (heights > floor_tolerance) & (heights <= robot_height)] = OBSTACLE
    labels[valid & (heights < -floor_tolerance)] = OBSTACLE
    return labels


def pose_matrix(position, rotation_matrix):
    """4x4 pose from a position and a row-major 3x3 rotation (as returned by Webots getOrientation)."""
    pose = np.eye(4)
    pose[:3, :3] = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    pose[:3, 3] = position
    return pose


def quaternion_pose(translation, quaternion):
    """4x4 pose from a translation and an (x, y, z, w) quaternion, e.g. a ROS TransformStamped."""
    x, y, z, w = quaternion
    pose = np.eye(4)
    pose[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    pose[:3, 3] = translation
    return pose
