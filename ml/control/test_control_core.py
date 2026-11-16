"""The Python ports in control_core.py reproduce the C++ control core (test_path_tracker.cpp,
test_limiter_and_checkers.cpp). If a C++ test there changes, mirror it here."""

import math

import numpy as np
import pytest

import control_core as cc


def straight(length, step=0.05, y=0.0):
    n = round(length / step)
    return np.array([[step * i, y, 0.0] for i in range(n + 1)])


def test_normalize_angle():
    assert cc.normalize_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert cc.normalize_angle(math.pi) == pytest.approx(-math.pi)  # [-pi, pi)


def test_to_robot_frame():
    x, y = cc.to_robot_frame((1.0, 1.0, math.pi / 2), 1.0, 2.0)
    assert x == pytest.approx(1.0) and y == pytest.approx(0.0, abs=1e-12)


def test_tracker_projection_and_cross_track_sign():
    t = cc.PathTracker(straight(4.0))
    t.update((1.0, 0.2, 0.0))
    assert t.progress() == pytest.approx(1.0)
    assert t.cross_track == pytest.approx(0.2)  # left of the path is positive
    assert t.remaining_length() == pytest.approx(3.0)
    t.update((1.5, -0.1, 0.0))
    assert t.cross_track == pytest.approx(-0.1)


def test_tracker_does_not_jump_back_on_a_loop():
    # Out along y = 0 and back along y = 0.3: near the start the return leg is closer, but beyond the window.
    out = straight(3.0)
    back = np.array([[3.0 - 0.05 * i, 0.3, math.pi] for i in range(1, 61)])
    t = cc.PathTracker(np.vstack([out, back]), 1.5)
    t.update((0.5, 0.2, 0.0))
    assert t.progress() == pytest.approx(0.5)


def test_point_ahead_clamps_to_goal():
    t = cc.PathTracker(straight(1.0))
    t.update((0.5, 0.0, 0.0))
    np.testing.assert_allclose(t.point_ahead(0.2)[:2], [0.7, 0.0], atol=1e-12)
    np.testing.assert_allclose(t.point_ahead(5.0), t.goal())


def test_limiter_accel_and_decel_are_asymmetric():
    v, w = cc.limit_command((0.5, 1.0), (0.0, 0.0), 0.05)
    assert v == pytest.approx(0.025) and w == pytest.approx(0.1)
    v, _ = cc.limit_command((0.0, 0.0), (0.4, 0.0), 0.05)
    assert v == pytest.approx(0.35)  # braking 1.0 m/s^2


def test_limiter_clamps_and_rejects_nan():
    assert cc.limit_command((2.0, 3.0), (0.5, 1.0), 0.05) == pytest.approx((0.5, 1.0))
    assert cc.limit_command((-0.3, 0.0), (0.0, 0.0), 0.05) == pytest.approx((0.0, 0.0))  # no reversing
    assert cc.limit_command((float("nan"), 0.0), (0.3, 0.0), 0.05) == (0.0, 0.0)
