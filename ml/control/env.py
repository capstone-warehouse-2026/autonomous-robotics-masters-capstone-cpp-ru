"""Training environment of the G4 learned controller (method B, PPO): a gymnasium.Env.

A procedural 2D scene instead of Webots: training needs millions of steps, the simulator gives hundreds per
second. What the policy sees is built exactly as in the robot (spec.path_features from the same PathTracker,
spec.build_observation), and its command passes the same CommandLimiter (control_core ports of the C++ core),
so the trained network runs unchanged in C++ (PpoController). The gap to Webots is covered by domain
randomization and checked by the isolated evaluation in Webots (docs/EXPERIMENTS.md).

Scenes come from a seed; the seed ranges of the splits are disjoint (docs/EXPERIMENTS.md): training uses
only "train", model selection only "val", the final comparison with the MPC only "test" and "ood".

One step (20 Hz, as the node): target = spec.action_to_target(action); command = limit_command(target,
previous command); after delay_steps periods the robot executes (gain_v * v, gain_w * w) with the unicycle
model integrated exactly over spec.DT (as MpcController::rollout); odometry = executed velocity + noise.
Domain randomization (training only): the gains (model mismatch, payload), the delay, odometry noise,
obstacle point noise and dropout.

Reward per step (weights below):
  + W_PROGRESS * increase of tracker.progress() (m)
  - W_TRACK * cross_track^2
  - W_SMOOTH * ((dv / (max_accel DT))^2 + (dw / (max_yaw_accel DT))^2)   change of the limited command
  - W_PROXIMITY * max(0, D_SAFE - clearance)^2
  - W_TIME
  + goal_potential(after) - goal_potential(before)                                  approach to the goal pose
goal_potential = -ramp * (W_GOAL_DISTANCE * distance to the goal + W_GOAL_HEADING * |yaw error to the goal|),
ramp = clip(1 - remaining / GOAL_ZONE, 0, 1): zero until the last GOAL_ZONE of the path, 1 at its end. A
difference of potentials (potential-based shaping, Ng et al. 1999) pays for every step towards the goal pose,
also after the end of the path where progress is saturated, and does not change which policy is optimal; a
per-step penalty on the yaw error instead taught the policy to stop short of the goal and wait for the time limit.
Terminal: collision (clearance < ROBOT_RADIUS) -> R_COLLISION, terminated; lost the path (|cross track| >
MAX_CROSS_TRACK, beyond it the tracker window and the reward lose their meaning) -> R_LOST, terminated;
FollowPath success of ADR 0004
(GoalChecker: distance to the goal <= 0.25 m, |yaw error| <= 0.2 rad, |v| < 0.05 m/s, |w| < 0.1 rad/s on the
measured velocity) -> R_GOAL, terminated; time limit -> truncated.
"""

from dataclasses import dataclass, field

import gymnasium as gym
import numpy as np

import control_core as cc
import spec

# Seed ranges of the splits (docs/EXPERIMENTS.md); disjoint.
SPLITS = {
    "train": range(0, 1000),
    "val": range(1000, 1100),
    "test": range(2000, 2020),
    "ood": range(3000, 3020),
}

TRACKER_WINDOW = 1.5  # m, the search window of the node's PathTracker

# Reward.
W_PROGRESS = 10.0
W_TRACK = 2.0
W_SMOOTH = 0.01
W_PROXIMITY = 20.0
W_GOAL_DISTANCE = 10.0  # per m, as W_PROGRESS: the last metres are worth the same as the path
W_GOAL_HEADING = 5.0  # per rad: turning 1 rad on the spot pays +5, about 0.25 per step at full yaw rate
W_TIME = 0.01
D_SAFE = 0.45  # m, same as MpcParams::d_safe
GOAL_ZONE = 0.5  # m, same as MpcParams::goal_zone
R_COLLISION = -50.0
R_GOAL = 50.0
# Off the path: a training and evaluation shortcut, not part of ADR 0004. Without it an untrained policy
# wanders metres away and the quadratic cross-track penalty gives returns of -1e4 that swamp everything else.
MAX_CROSS_TRACK = 1.0  # m
R_LOST = -50.0

# FollowPath success, docs/adr/0004 (GoalTolerance).
GOAL_XY = 0.25  # m
GOAL_YAW = 0.2  # rad
GOAL_SPEED = 0.05  # m/s
GOAL_YAW_RATE = 0.1  # rad/s

# Episode length: time to drive the path at TIME_LIMIT_SPEED plus TIME_LIMIT_EXTRA.
TIME_LIMIT_SPEED = 0.15  # m/s
TIME_LIMIT_EXTRA = 10.0  # s


@dataclass
class Scenario:
    """One scene in the world frame."""

    path: np.ndarray  # (M, 3) poses (x, y, yaw) from the start to the goal; the last yaw is the goal yaw
    obstacles: np.ndarray  # (K, 2) static obstacle points (occupied cells, not inflated)
    start: tuple  # (x, y, yaw) of the robot
    # Dynamic obstacles: discs of `radius` moving with constant velocity from `position` (world frame).
    dynamic_positions: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    dynamic_velocities: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    dynamic_radius: float = 0.2


@dataclass
class Randomization:
    """Per-episode physics and sensing; Randomization() is the nominal robot."""

    gain_v: float = 1.0
    gain_w: float = 1.0
    delay_steps: int = 0
    odom_noise: float = 0.0  # m/s and rad/s, std
    obstacle_noise: float = 0.0  # m, std
    obstacle_dropout: float = 0.0  # probability to lose a point


def path_length(path):
    return float(np.sum(np.hypot(*np.diff(np.asarray(path)[:, :2], axis=0).T)))


# Scene generation: lengths in m, angles in rad.
PIECES = (3, 5)  # straight pieces of the path, inclusive
PIECE_LENGTH = (1.0, 2.0)
MAX_TURN = np.deg2rad(80.0)  # between pieces
PATH_STEP = (0.05, 0.1)  # resampling, as the paths of G3
GOAL_TURN_PROBABILITY = 0.5  # the goal yaw differs from the last segment: trains the turn on the spot
BOXES = (2, 6)  # inclusive
BOX_HALF = (0.15, 0.4)
PATH_MARGIN = spec.ROBOT_RADIUS + 0.1  # the global planner keeps the path this free of static points
NARROW_PROBABILITY = 0.3
NARROW_GAP = (0.8, 1.0)  # free width between the two boxes of a narrow passage
DYNAMIC_PROBABILITY = 0.3
DYNAMIC_SPEED = (0.2, 0.5)  # m/s
NOMINAL_SPEED = 0.4  # m/s, to time a dynamic obstacle so that it crosses ahead of the robot
START_OFFSET = 0.2
START_YAW_ERROR = 0.3
START_MARGIN = spec.ROBOT_RADIUS + 0.3  # free space around the start pose


def _resample(corners, step):
    """Polyline through `corners` with points at most `step` apart (both corners kept)."""
    out = [corners[0]]
    for a, b in zip(corners[:-1], corners[1:]):
        n = max(1, int(np.ceil(np.hypot(*(b - a)) / step)))
        out.extend(a + (b - a) * i / n for i in range(1, n + 1))
    return np.array(out)


def _box(center, half, step=0.05):
    """Perimeter points of an axis-aligned square box: the occupied cells /perception/obstacles would give."""
    t = np.linspace(-half, half, max(2, int(np.ceil(2 * half / step)) + 1))
    cx, cy = center
    return np.concatenate([
        np.column_stack([cx + t, np.full_like(t, cy - half)]),
        np.column_stack([cx + t, np.full_like(t, cy + half)]),
        np.column_stack([np.full_like(t, cx - half), cy + t]),
        np.column_stack([np.full_like(t, cx + half), cy + t]),
    ])


def _min_distance(points, xy):
    """Smallest distance between two point sets, (K, 2) and (M, 2)."""
    return float(np.min(np.hypot(*(points[:, None, :] - xy[None, :, :]).transpose(2, 0, 1))))


def _normal(xy, k):
    """Unit left normal of the path at point k (central difference)."""
    d = xy[min(k + 1, len(xy) - 1)] - xy[max(k - 1, 0)]
    return np.array([-d[1], d[0]]) / (np.hypot(*d) + 1e-12)


def generate_scenario(rng):
    """Random scene from a numpy Generator; the same generator state gives the same scene."""
    # Path: a random walk of straight pieces, resampled densely; each pose looks along its segment.
    heading = rng.uniform(-np.pi, np.pi)
    corners = [np.zeros(2)]
    for i in range(int(rng.integers(PIECES[0], PIECES[1] + 1))):
        if i > 0:
            heading += rng.uniform(-MAX_TURN, MAX_TURN)
        corners.append(corners[-1] + rng.uniform(*PIECE_LENGTH) * np.array([np.cos(heading), np.sin(heading)]))
    xy = _resample(np.array(corners), rng.uniform(*PATH_STEP))
    d = np.diff(xy, axis=0)
    segment_yaw = np.arctan2(d[:, 1], d[:, 0])
    yaw = np.concatenate([segment_yaw[:1], segment_yaw])  # pose i looks along the segment that ends at it
    if rng.uniform() < GOAL_TURN_PROBABILITY:
        yaw[-1] = cc.normalize_angle(yaw[-1] + rng.uniform(-np.pi / 2, np.pi / 2))
    path = np.column_stack([xy, yaw])
    # The robot near the first pose, slightly off and misaligned, as after the previous task.
    start = (
        float(xy[0, 0] + rng.uniform(-START_OFFSET, START_OFFSET)),
        float(xy[0, 1] + rng.uniform(-START_OFFSET, START_OFFSET)),
        float(cc.normalize_angle(yaw[0] + rng.uniform(-START_YAW_ERROR, START_YAW_ERROR))),
    )
    start_xy = np.array(start[:2])

    # Static boxes beside the path. A box is dropped if any of its points is within PATH_MARGIN of the path
    # (the global planner would not have planned through it) or within START_MARGIN of the start (an episode
    # must not begin in a collision the robot cannot avoid).
    boxes = []
    for _ in range(int(rng.integers(BOXES[0], BOXES[1] + 1))):
        k = int(rng.integers(5, len(xy) - 1))
        half = rng.uniform(*BOX_HALF)
        # Centre distance so that even the nearest corner (half * sqrt 2) stays clear, plus some slack.
        offset = PATH_MARGIN + half * np.sqrt(2) + rng.uniform(0.05, 0.6)
        boxes.append(_box(xy[k] + rng.choice([-1.0, 1.0]) * offset * _normal(xy, k), half))
    if rng.uniform() < NARROW_PROBABILITY:
        # Narrow passage: two boxes facing each other across the path.
        k = int(rng.integers(len(xy) // 4, len(xy) - 1))
        half = rng.uniform(*BOX_HALF)
        offset = rng.uniform(*NARROW_GAP) / 2 + half * np.sqrt(2)
        boxes += [_box(xy[k] + side * offset * _normal(xy, k), half) for side in (-1.0, 1.0)]
    boxes = [b for b in boxes if _min_distance(b, xy) >= PATH_MARGIN
             and _min_distance(b, start_xy[None]) >= START_MARGIN]
    obstacles = np.concatenate(boxes) if boxes else np.zeros((0, 2))

    # Dynamic obstacle: starts beside the path and walks across it, timed to arrive about when the robot does.
    positions, velocities = np.zeros((0, 2)), np.zeros((0, 2))
    if rng.uniform() < DYNAMIC_PROBABILITY:
        k = int(rng.integers(len(xy) // 3, len(xy) - 1))
        arc = float(np.sum(np.hypot(*d[:k].T)))  # path length to the crossing point
        speed = rng.uniform(*DYNAMIC_SPEED)
        side = rng.choice([-1.0, 1.0])
        distance = np.clip(speed * arc / NOMINAL_SPEED, 1.0, 4.0)
        position = xy[k] + side * distance * _normal(xy, k)
        # A path that turns back may pass the start: the disc must not begin on top of the robot.
        if np.hypot(*(position - start_xy)) - Scenario.dynamic_radius >= START_MARGIN:
            positions = position[None, :]
            velocities = (-side * speed * _normal(xy, k))[None, :]

    return Scenario(path=path, obstacles=obstacles, start=start, dynamic_positions=positions,
                    dynamic_velocities=velocities)


def sample_randomization(rng):
    """Domain randomization for one training episode."""
    return Randomization(
        gain_v=rng.uniform(0.8, 1.2),  # the +-20 % model mismatch of the test, and payload
        gain_w=rng.uniform(0.8, 1.2),
        delay_steps=int(rng.integers(0, 3)),  # 0..2 periods, up to 100 ms
        odom_noise=rng.uniform(0.0, 0.02),
        obstacle_noise=rng.uniform(0.0, 0.02),
        obstacle_dropout=rng.uniform(0.0, 0.1),
    )


class WarehouseEnv(gym.Env):
    """reset(seed=...) draws a scene of the split; options={"scenario": Scenario, "randomize": bool} replace
    the random scene and switch domain randomization off or on (tests, evaluation)."""

    metadata = {"render_modes": []}

    def __init__(self, split="train", domain_randomization=True):
        if split not in SPLITS:
            raise ValueError(f"unknown split {split!r}")
        self.split = split
        self.domain_randomization = domain_randomization
        self.limits = cc.CommandLimits()
        self.observation_space = gym.spaces.Box(-10.0, 10.0, shape=(spec.OBS_SIZE,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(spec.ACTION_SIZE,), dtype=np.float32)

    # ---------------------------------------------------------------- gymnasium API

    def reset(self, *, seed=None, options=None):
        options = options or {}
        if "scenario" not in options:
            if seed is None or seed not in SPLITS[self.split]:
                raise ValueError(f"seed {seed} is not in split {self.split!r} {SPLITS[self.split]}")
        super().reset(seed=seed)
        rng = np.random.default_rng(seed)
        self.scenario = options["scenario"] if "scenario" in options else generate_scenario(rng)
        randomize = options.get("randomize", self.domain_randomization)
        self.randomization = sample_randomization(rng) if randomize else Randomization()

        self.pose = np.array(self.scenario.start, dtype=float)  # true (x, y, yaw)
        self.velocity = np.zeros(2)  # executed (v, w)
        self.measured = np.zeros(2)  # odometry (v, w)
        self.command = np.zeros(2)  # last limited command
        self.pending = [np.zeros(2) for _ in range(self.randomization.delay_steps)]  # delayed commands
        self.tracker = cc.PathTracker(self.scenario.path, TRACKER_WINDOW)
        self.tracker.update(self.pose)
        # Own copy: _advance moves the dynamic obstacles, the Scenario of the caller stays as given.
        self.dynamic_positions = np.array(self.scenario.dynamic_positions, dtype=float).reshape(-1, 2)
        self.steps = 0
        self.max_steps = int((path_length(self.scenario.path) / TIME_LIMIT_SPEED + TIME_LIMIT_EXTRA) / spec.DT)
        self.rng = rng
        self.collision = self.success = self.lost = False
        self.clearance = self._clearance()
        self.goal_potential = self._goal_potential()
        return self._observe(), self._info()

    def step(self, action):
        previous_command = self.command.copy()
        previous_progress = self.tracker.progress()
        target = spec.action_to_target(action)
        self.command = np.array(cc.limit_command(target, tuple(self.command), spec.DT, self.limits))
        self._advance(self.command)
        self.tracker.update(self.pose)
        self.steps += 1
        reward, terminated = self._reward(previous_command, previous_progress)
        truncated = not terminated and self.steps >= self.max_steps
        return self._observe(), reward, terminated, truncated, self._info()

    # ---------------------------------------------------------------- dynamics and sensing

    def _advance(self, command):
        """One control period: delay queue, actuation gains, exact unicycle step of the true pose, odometry
        with noise, dynamic obstacles moved."""
        r = self.randomization
        if r.delay_steps > 0:
            # FIFO of delay_steps commands: the one applied now was limited delay_steps periods ago.
            self.pending.append(np.array(command, dtype=float))
            applied = self.pending.pop(0)
        else:
            applied = command
        v, w = r.gain_v * applied[0], r.gain_w * applied[1]

        # Exact integration over DT with constant (v, w): a circular arc, a straight line for w = 0 (the same
        # formulas as MpcController::rollout, so both methods are trained and planned on one model).
        x, y, yaw = self.pose
        yaw1 = yaw + w * spec.DT
        if abs(w) < 1e-9:
            x += v * spec.DT * np.cos(yaw)
            y += v * spec.DT * np.sin(yaw)
        else:
            x += v / w * (np.sin(yaw1) - np.sin(yaw))
            y -= v / w * (np.cos(yaw1) - np.cos(yaw))
        self.pose = np.array([x, y, cc.normalize_angle(yaw1)])

        self.velocity = np.array([v, w])
        self.measured = self.velocity.copy()
        if r.odom_noise > 0:
            self.measured += self.rng.normal(0.0, r.odom_noise, 2)
        self.dynamic_positions = self.dynamic_positions + self.scenario.dynamic_velocities * spec.DT

    def _obstacle_points(self, noisy):
        """World-frame obstacle points: static points and points every ~0.05 m on the dynamic discs; with
        noise and dropout from self.randomization if `noisy`."""
        sc = self.scenario
        parts = [np.asarray(sc.obstacles, dtype=float).reshape(-1, 2)]
        if len(self.dynamic_positions):
            n = max(8, int(2 * np.pi * sc.dynamic_radius / 0.05))
            a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
            ring = sc.dynamic_radius * np.column_stack([np.cos(a), np.sin(a)])
            parts += [c + ring for c in self.dynamic_positions]
        points = np.concatenate(parts)
        if noisy and len(points):
            r = self.randomization
            if r.obstacle_dropout > 0:
                points = points[self.rng.uniform(size=len(points)) >= r.obstacle_dropout]
            if r.obstacle_noise > 0:
                points = points + self.rng.normal(0.0, r.obstacle_noise, points.shape)
        return points

    def _observe(self):
        """spec.build_observation from the tracker and the obstacles in the robot frame."""
        points, remaining, goal_yaw = spec.path_features(self.tracker, self.pose)
        # Obstacles in base_link of the TRUE pose: localization noise belongs to G1, not to this method.
        # control_core.to_robot_frame, vectorized: this runs for hundreds of points every step.
        world = self._obstacle_points(noisy=True)
        dx, dy = world[:, 0] - self.pose[0], world[:, 1] - self.pose[1]
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        local = np.column_stack([c * dx + s * dy, -s * dx + c * dy])
        local = local[np.hypot(local[:, 0], local[:, 1]) <= spec.RANGE_MAX]
        return spec.build_observation(points, remaining, goal_yaw, local, *self.measured)

    # ---------------------------------------------------------------- reward

    def _clearance(self):
        """Distance from the robot centre to the nearest obstacle, without sensor noise (ground truth)."""
        x, y = self.pose[0], self.pose[1]
        sc = self.scenario
        d = np.inf
        if len(sc.obstacles):
            d = float(np.min(np.hypot(sc.obstacles[:, 0] - x, sc.obstacles[:, 1] - y)))
        for cx, cy in self.dynamic_positions:
            d = min(d, float(np.hypot(cx - x, cy - y)) - sc.dynamic_radius)  # to the disc surface
        return d

    def _goal_potential(self):
        """Shaping potential of the goal pose, see the module docstring; 0 far from the goal."""
        ramp = min(max(1.0 - self.tracker.remaining_length() / GOAL_ZONE, 0.0), 1.0)
        if ramp == 0.0:
            return 0.0
        goal = self.tracker.goal()
        distance = np.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1])
        yaw_error = abs(cc.normalize_angle(goal[2] - self.pose[2]))
        return -ramp * (W_GOAL_DISTANCE * distance + W_GOAL_HEADING * yaw_error)

    def _reward(self, previous_command, previous_progress):
        """(reward, terminated) from the module docstring; sets self.collision, self.success, self.lost and
        self.clearance."""
        x, y, yaw = self.pose
        goal = self.tracker.goal()
        yaw_error = cc.normalize_angle(goal[2] - yaw)
        self.clearance = self._clearance()

        # Collision first: touching an obstacle at the goal is still a collision.
        self.collision = self.clearance < spec.ROBOT_RADIUS
        self.lost = not self.collision and abs(self.tracker.cross_track) > MAX_CROSS_TRACK
        # FollowPath success of ADR 0004 (GoalChecker), on the measured velocity as in the node.
        self.success = (
            not self.collision
            and not self.lost
            and np.hypot(goal[0] - x, goal[1] - y) <= GOAL_XY
            and abs(yaw_error) <= GOAL_YAW
            and abs(self.measured[0]) < GOAL_SPEED
            and abs(self.measured[1]) < GOAL_YAW_RATE
        )

        # Change of the limited command relative to its largest possible change in one period: 1.0 = full
        # acceleration. Smoothness is measured on what the robot gets, not on the raw network output.
        dv = (self.command[0] - previous_command[0]) / (self.limits.max_accel * spec.DT)
        dw = (self.command[1] - previous_command[1]) / (self.limits.max_yaw_accel * spec.DT)
        reward = (
            W_PROGRESS * (self.tracker.progress() - previous_progress)
            - W_TRACK * self.tracker.cross_track ** 2
            - W_SMOOTH * (dv ** 2 + dw ** 2)
            - W_PROXIMITY * max(0.0, D_SAFE - self.clearance) ** 2
            - W_TIME
        )
        potential = self._goal_potential()
        reward += potential - self.goal_potential
        self.goal_potential = potential

        if self.collision:
            return reward + R_COLLISION, True
        if self.lost:
            return reward + R_LOST, True
        if self.success:
            return reward + R_GOAL, True
        return reward, False

    def _info(self):
        """Per-step diagnostics for evaluation; keys used by evaluate.py and the tests."""
        r = self.randomization
        return {
            "collision": bool(self.collision),
            "success": bool(self.success),
            "lost": bool(self.lost),
            "progress": float(self.tracker.progress()),
            "cross_track": float(self.tracker.cross_track),
            "clearance": float(self.clearance),
            "gain_v": float(r.gain_v),
            "gain_w": float(r.gain_w),
            "delay_steps": int(r.delay_steps),
        }
