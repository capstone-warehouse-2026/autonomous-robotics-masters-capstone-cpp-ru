"""Python ports of the G4 control core used by the training environment.

PathTracker and CommandLimiter are line-by-line ports of ros2_ws/src/capstone_control (path_tracker.cpp,
command_limiter.cpp; docs/adr/0004). The policy is trained behind exactly the same path projection and
the same actuation limits it meets in the robot. test_control_core.py checks them against the numbers of
the C++ tests; change both sides together.
"""

import math
from dataclasses import dataclass

import numpy as np


def normalize_angle(a):
    """Wraps an angle to [-pi, pi), as geometry.hpp."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def to_robot_frame(robot, wx, wy):
    """World point (wx, wy) in the frame of robot = (x, y, yaw): x forward, y left."""
    dx, dy = wx - robot[0], wy - robot[1]
    c, s = math.cos(robot[2]), math.sin(robot[2])
    return c * dx + s * dy, -s * dx + c * dy


class PathTracker:
    """Projection of the robot onto a fixed path, forward only within `search_window` (path_tracker.cpp)."""

    def __init__(self, path, search_window=1.5):
        self.path = np.asarray(path, dtype=float).reshape(-1, 3)  # (x, y, yaw) per pose
        self.search_window = search_window
        steps = np.hypot(*np.diff(self.path[:, :2], axis=0).T) if len(self.path) > 1 else np.zeros(0)
        self.cumulative = np.concatenate([[0.0], np.cumsum(steps)])
        self.segment = 0
        self.s = 0.0
        self.cross_track = 0.0

    def goal(self):
        return self.path[-1]

    def total_length(self):
        return float(self.cumulative[-1]) if len(self.cumulative) else 0.0

    def progress(self):
        return self.s

    def remaining_length(self):
        return self.total_length() - self.s

    def update(self, robot):
        path = self.path
        if len(path) == 0:
            return
        if len(path) == 1:
            self.s = 0.0
            self.cross_track = math.hypot(robot[0] - path[0, 0], robot[1] - path[0, 1])
            return
        best_dist, best_segment, best_s, best_cross = math.inf, self.segment, self.s, self.cross_track
        for i in range(self.segment, len(path) - 1):
            if i > self.segment and self.cumulative[i] > self.s + self.search_window:
                break
            ax, ay = path[i, 0], path[i, 1]
            dx, dy = path[i + 1, 0] - ax, path[i + 1, 1] - ay
            len2 = dx * dx + dy * dy
            if len2 < 1e-12:
                continue
            length = math.sqrt(len2)
            t = min(max(((robot[0] - ax) * dx + (robot[1] - ay) * dy) / len2, 0.0), 1.0)
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(robot[0] - px, robot[1] - py)
            if d < best_dist:
                best_dist, best_segment = d, i
                best_s = self.cumulative[i] + t * length
                cross = (dx * (robot[1] - ay) - dy * (robot[0] - ax)) / length
                best_cross = math.copysign(d, cross)
        if math.isfinite(best_dist):
            self.segment, self.s, self.cross_track = best_segment, float(best_s), float(best_cross)

    def point_ahead(self, ahead):
        """(x, y, yaw) on the path `ahead` metres past the projection, clamped to the end."""
        path = self.path
        if len(path) <= 1:
            return path[0].copy() if len(path) else np.zeros(3)
        target = min(self.s + max(ahead, 0.0), self.total_length())
        if target >= self.total_length():
            return path[-1].copy()
        j = int(np.searchsorted(self.cumulative, target, side="right"))
        a, b = path[j - 1], path[j]
        seg = self.cumulative[j] - self.cumulative[j - 1]
        t = (target - self.cumulative[j - 1]) / seg if seg > 1e-12 else 0.0
        return np.array([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])])


@dataclass
class CommandLimits:
    """command_limiter.hpp defaults (ADR 0004)."""

    max_speed: float = 0.5
    max_reverse_speed: float = 0.0
    max_yaw_rate: float = 1.0
    max_accel: float = 0.5
    max_decel: float = 1.0
    max_yaw_accel: float = 2.0


def _rate_limit(desired, previous, up, down, dt):
    slowing = abs(desired) < abs(previous) and desired * previous >= 0.0
    step = (down if slowing else up) * dt
    return previous + min(max(desired - previous, -step), step)


def limit_command(desired, previous, dt, limits=CommandLimits()):
    """CommandLimiter::limit: (v, w) desired and previous command -> limited (v, w)."""
    if not (math.isfinite(desired[0]) and math.isfinite(desired[1])):
        return 0.0, 0.0
    prev = previous if (math.isfinite(previous[0]) and math.isfinite(previous[1])) else (0.0, 0.0)
    step_dt = dt if (math.isfinite(dt) and dt > 0.0) else 0.0
    v = min(max(desired[0], -limits.max_reverse_speed), limits.max_speed)
    w = min(max(desired[1], -limits.max_yaw_rate), limits.max_yaw_rate)
    v = _rate_limit(v, prev[0], limits.max_accel, limits.max_decel, step_dt)
    w = _rate_limit(w, prev[1], limits.max_yaw_accel, limits.max_yaw_accel, step_dt)
    v = min(max(v, -limits.max_reverse_speed), limits.max_speed)
    w = min(max(w, -limits.max_yaw_rate), limits.max_yaw_rate)
    return v, w
