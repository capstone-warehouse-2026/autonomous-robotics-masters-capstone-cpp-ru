"""Unit tests of labels.py: python3 -m unittest discover ml/perception"""

import unittest

import numpy as np

from labels import FREE, OBSTACLE, UNKNOWN, compute_labels, pixel_heights, quaternion_pose

# 5x5 camera looking along world +x from 1 m above the floor; optical axes mapped to the world.
CAMERA = {'fx': 1000.0, 'fy': 1000.0, 'cx': 2.0, 'cy': 2.0}
WORLD_FROM_CAMERA = np.array([
    [0.0, 0.0, 1.0, 0.0],    # optical z (forward) -> world x
    [-1.0, 0.0, 0.0, 0.0],   # optical x (right)   -> world -y
    [0.0, -1.0, 0.0, 1.0],   # optical y (down)    -> world -z, camera 1 m high
    [0.0, 0.0, 0.0, 1.0],
])


def camera_looking_down():
    """Camera 1 m above the floor looking straight down: every pixel at depth 1 m is floor."""
    pose = np.eye(4)
    pose[:3, :3] = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
    pose[2, 3] = 1.0
    return pose


class LabelsTest(unittest.TestCase):

    def test_floor_seen_from_above_is_free(self):
        depth = np.full((5, 5), 1.0, dtype=np.float32)
        labels = compute_labels(depth, CAMERA, camera_looking_down())
        self.assertTrue((labels == FREE).all())

    def test_heights_of_horizontal_camera(self):
        depth = np.full((5, 5), 2.0, dtype=np.float32)
        heights = pixel_heights(depth, CAMERA, WORLD_FROM_CAMERA)
        self.assertAlmostEqual(heights[2, 2], 1.0)                   # centre ray at camera height
        self.assertAlmostEqual(heights[0, 2], 1.0 + 2 * 2.0 / 1000)  # upper rows are higher

    def test_obstacle_above_robot_and_missing_depth(self):
        depth = np.full((5, 5), 2.0, dtype=np.float32)
        depth[0, 0] = np.nan
        labels = compute_labels(depth, CAMERA, WORLD_FROM_CAMERA, robot_height=1.0)
        self.assertEqual(labels[0, 0], UNKNOWN)   # no depth
        self.assertEqual(labels[4, 2], OBSTACLE)  # just below 1 m: inside the robot height
        self.assertEqual(labels[0, 2], UNKNOWN)   # above the robot: it passes under

    def test_point_below_floor_is_obstacle(self):
        pose = camera_looking_down()
        depth = np.full((5, 5), 1.2, dtype=np.float32)  # 20 cm below the floor
        self.assertTrue((compute_labels(depth, CAMERA, pose) == OBSTACLE).all())

    def test_quaternion_pose_rotates_about_z(self):
        half = np.sqrt(0.5)
        pose = quaternion_pose([1.0, 2.0, 3.0], [0.0, 0.0, half, half])  # +90 deg about z
        np.testing.assert_allclose(pose[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(pose[:3, 3], [1.0, 2.0, 3.0])


if __name__ == '__main__':
    unittest.main()
