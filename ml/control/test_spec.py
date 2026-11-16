"""Specification tests of spec.py; test_ppo_observation.cpp checks the same cases in C++."""

import math

import numpy as np
import pytest

import control_core as cc
import spec

NO_OBSTACLES = np.zeros((0, 2))


def ahead_points(y=0.0):
    """Points 0.2..2.0 m straight ahead at lateral offset y."""
    return np.array([[0.2 * (i + 1), y] for i in range(spec.PATH_POINTS)])


def obs_of(points=None, remaining=3.0, goal_yaw=0.0, obstacles=NO_OBSTACLES, v=0.0, w=0.0):
    return spec.build_observation(ahead_points() if points is None else points, remaining, goal_yaw, obstacles, v, w)


def path_xy(obs):
    return obs[: 2 * spec.PATH_POINTS].reshape(spec.PATH_POINTS, 2) * spec.PATH_SCALE


def sectors(obs):
    return obs[23:59] * spec.RANGE_MAX


def test_layout_and_dtype():
    obs = obs_of(v=0.25, w=-0.5)
    assert obs.shape == (spec.OBS_SIZE,) and spec.OBS_SIZE == 61
    assert obs.dtype == np.float32
    assert obs[59] == pytest.approx(0.5) and obs[60] == pytest.approx(-0.5)


def test_path_points_are_scaled():
    pts = path_xy(obs_of(points=ahead_points(0.3)))
    np.testing.assert_allclose(pts[:, 0], [0.2 * (i + 1) for i in range(10)], atol=1e-6)
    np.testing.assert_allclose(pts[:, 1], 0.3, atol=1e-6)


def test_remaining_is_scaled_and_capped():
    assert obs_of(remaining=0.5)[20] == pytest.approx(0.1)
    assert obs_of(remaining=8.0)[20] == pytest.approx(1.0)


def test_goal_yaw_as_cos_sin():
    obs = obs_of(goal_yaw=math.pi / 2)
    assert obs[21] == pytest.approx(0.0, abs=1e-7) and obs[22] == pytest.approx(1.0)
    obs = obs_of(goal_yaw=-math.pi)
    assert obs[21] == pytest.approx(-1.0) and obs[22] == pytest.approx(0.0, abs=1e-7)


def test_sectors_empty_and_bearing():
    # Bearings away from sector borders, so that rounding cannot move a point to the next sector.
    obstacles = np.array([[1.5, 0.1], [2.0, 0.15], [-0.1, 1.2], [-0.9, -0.001], [5.0, 5.0]])
    r = sectors(obs_of(obstacles=obstacles))
    assert r[18] == pytest.approx(math.hypot(1.5, 0.1))  # bearing ~4 deg -> [0, 10 deg); the nearer point
    assert r[27] == pytest.approx(math.hypot(0.1, 1.2))  # bearing ~95 deg -> [90, 100 deg)
    assert r[0] == pytest.approx(math.hypot(0.9, 0.001))  # bearing just above -180 deg
    assert r[31] == pytest.approx(spec.RANGE_MAX)  # (5, 5) is beyond RANGE_MAX
    assert sum(1 for x in r if x < spec.RANGE_MAX) == 3


def test_bearing_pi_goes_to_last_sector():
    assert sectors(obs_of(obstacles=np.array([[-1.0, 0.0]])))[35] == pytest.approx(1.0)


def test_action_to_target():
    assert spec.action_to_target([1.0, 0.0]) == pytest.approx((0.5, 0.0))
    assert spec.action_to_target([-1.0, -1.0]) == pytest.approx((0.0, -1.0))
    assert spec.action_to_target([0.0, 0.5]) == pytest.approx((0.25, 0.5))
    assert spec.action_to_target([5.0, -5.0]) == pytest.approx((0.5, -1.0))  # clipped


def test_path_features_from_tracker():
    # Path along world +x at y = 1; the robot stands on it at x = 1, facing +y: the path runs to its right.
    path = np.array([[0.1 * i, 1.0, 0.0] for i in range(51)])
    tracker = cc.PathTracker(path)
    robot = (1.0, 1.0, math.pi / 2)
    tracker.update(robot)
    points, remaining, goal_yaw = spec.path_features(tracker, robot)
    np.testing.assert_allclose(points[0], [0.0, -0.2], atol=1e-9)
    np.testing.assert_allclose(points[9], [0.0, -2.0], atol=1e-9)
    assert remaining == pytest.approx(4.0)
    assert goal_yaw == pytest.approx(-math.pi / 2)
