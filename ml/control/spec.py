"""Observation and action specification of the G4 learned controller (method B, PPO).

This file is the contract between training (Python) and the robot (C++): ppo_observation.hpp in
ros2_ws/src/capstone_control must reproduce build_observation() and action_to_target() literally;
test_spec.py and test_ppo_observation.cpp check the same numbers. Changing anything here invalidates trained
models: retrain and re-export.

Both methods see the same information (projects/04_control.md, controller.hpp ControlInput): the path through
the PathTracker of the node, the obstacles in base_link and the measured velocity. No ground truth, no future
obstacles. The caller extracts the primitive inputs below from a PathTracker (path_features()), in Python
from control_core.PathTracker, in C++ from capstone_control::PathTracker; both behave identically.

Observation, float32, OBS_SIZE = 61 values, in this order:
  [0, 20)   PATH_POINTS points tracker.point_ahead((i + 1) * PATH_SPACING), i = 0..9, in the robot frame
            (x forward, y left), divided by PATH_SCALE, interleaved x0, y0, x1, y1, ...
  [20]      min(tracker.remaining_length(), REMAINING_MAX) / REMAINING_MAX
  [21, 23)  cos and sin of the goal yaw relative to the robot yaw (the final turn on the spot)
  [23, 59)  SECTORS obstacle ranges: sector i covers bearings [-pi + i * 2pi / SECTORS, -pi + (i + 1) * 2pi /
            SECTORS); the value is min(range of obstacle points in it, RANGE_MAX) / RANGE_MAX, 1.0 if empty.
  [59, 61)  v / V_MAX, w / W_MAX of the measured velocity.

Action: 2 floats; each is clipped to [-1, 1]. Target v = (a0 + 1) / 2 * V_MAX, target w = a1 * W_MAX. The
CommandLimiter of the node (control_core.limit_command in training) then applies the acceleration limits, the
same for every controller (docs/adr/0004).
"""

import math

import numpy as np

# Robot limits, docs/adr/0004 (CommandLimits); identical to MpcParams.
V_MAX = 0.5  # m/s
W_MAX = 1.0  # rad/s
DT = 0.05  # s, control period (20 Hz)
ROBOT_RADIUS = 0.25  # m

# Path features.
PATH_POINTS = 10
PATH_SPACING = 0.2  # m, the points cover 2 m ahead
PATH_SCALE = 2.0  # m
REMAINING_MAX = 5.0  # m

# Obstacle features.
SECTORS = 36  # 10 degrees each
RANGE_MAX = 3.0  # m

OBS_SIZE = 2 * PATH_POINTS + 1 + 2 + SECTORS + 2
ACTION_SIZE = 2


def path_features(tracker, robot):
    """Primitive path inputs from an updated control_core.PathTracker and the robot pose (x, y, yaw):
    (points (PATH_POINTS, 2) in the robot frame, remaining length, goal yaw relative to the robot).
    Provided, not part of the TODO: it only calls the tracker, as the C++ side does."""
    from control_core import normalize_angle, to_robot_frame

    points = np.array(
        [to_robot_frame(robot, *tracker.point_ahead((i + 1) * PATH_SPACING)[:2]) for i in range(PATH_POINTS)]
    )
    return points, tracker.remaining_length(), normalize_angle(tracker.goal()[2] - robot[2])


def build_observation(points, remaining, goal_yaw, obstacles, v, w):
    """Observation vector (float32, OBS_SIZE).

    points: (PATH_POINTS, 2) path points in base_link (from path_features()).
    remaining: path length left, m.
    goal_yaw: goal heading relative to the robot, rad.
    obstacles: (K, 2) array (K may be 0) of obstacle points in base_link.
    v, w: measured velocities.
    """
    obs_points = np.asarray(points, dtype=np.float64).reshape(PATH_POINTS, 2).ravel() / PATH_SCALE
    obs_remaining = min(remaining, REMAINING_MAX) / REMAINING_MAX
    obs_goal = np.array([math.cos(goal_yaw), math.sin(goal_yaw)], dtype=np.float64)
    
    sectors = np.full(SECTORS, RANGE_MAX, dtype=np.float64)
    if obstacles is not None and len(obstacles) > 0:
        obs_arr = np.asarray(obstacles, dtype=np.float64)
        x = obs_arr[:, 0]
        y = obs_arr[:, 1]
        ranges = np.hypot(x, y)
        bearings = np.arctan2(y, x)
        sector_width = 2.0 * math.pi / SECTORS
        indices = np.floor((bearings + math.pi) / sector_width).astype(np.int32)
        indices = np.clip(indices, 0, SECTORS - 1)
        np.minimum.at(sectors, indices, ranges)
    sectors = np.minimum(sectors, RANGE_MAX)
    obs_sectors = sectors / RANGE_MAX

    obs_vel = np.array([v / V_MAX, w / W_MAX], dtype=np.float64)

    obs = np.concatenate([
        obs_points,
        [obs_remaining],
        obs_goal,
        obs_sectors,
        obs_vel,
    ])
    return obs.astype(np.float32)


def action_to_target(action):
    """(v, w) target command from a policy action (2 floats), before the CommandLimiter."""
    a = np.clip(action, -1.0, 1.0)
    v = (a[0] + 1.0) / 2.0 * V_MAX
    w = a[1] * W_MAX
    return float(v), float(w)
