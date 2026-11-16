"""Specification tests of the training environment (env.py)."""

import math

import numpy as np
import pytest

import control_core as cc
import env as E
import spec


def straight(length=5.0, y=0.0, goal_yaw=0.0):
    n = round(length / 0.1)
    path = np.array([[0.1 * i, y, 0.0] for i in range(n + 1)])
    path[-1, 2] = goal_yaw
    return path


def scenario(path=None, obstacles=None, start=(0.0, 0.0, 0.0)):
    return E.Scenario(
        path=straight() if path is None else path,
        obstacles=np.zeros((0, 2)) if obstacles is None else np.asarray(obstacles, dtype=float),
        start=start,
    )


def nominal(sc):
    e = E.WarehouseEnv("train")
    obs, info = e.reset(options={"scenario": sc, "randomize": False})
    return e, obs, info


def run(e, policy, limit=2000):
    """Steps until the episode ends; returns the last (reward, terminated, truncated, info)."""
    for _ in range(limit):
        _, r, terminated, truncated, info = e.step(policy(e))
        if terminated or truncated:
            return r, terminated, truncated, info
    raise AssertionError("episode did not end")


# ---------------------------------------------------------------- splits and spaces


def test_splits_are_disjoint():
    names = list(E.SPLITS)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            assert not set(E.SPLITS[a]) & set(E.SPLITS[b]), (a, b)


def test_seed_must_belong_to_split():
    with pytest.raises(ValueError):
        E.WarehouseEnv("train").reset(seed=2000)  # a test seed
    with pytest.raises(ValueError):
        E.WarehouseEnv("val").reset(seed=0)
    with pytest.raises(ValueError):
        E.WarehouseEnv("train").reset()  # no seed


def test_spaces():
    e, obs, _ = nominal(scenario())
    assert e.observation_space.shape == (spec.OBS_SIZE,)
    assert e.action_space.shape == (2,)
    assert obs.shape == (spec.OBS_SIZE,) and obs.dtype == np.float32
    assert e.observation_space.contains(obs)


# ---------------------------------------------------------------- generated scenes


def test_reset_is_deterministic_per_seed():
    a, _ = E.WarehouseEnv("train").reset(seed=7)
    b, _ = E.WarehouseEnv("train").reset(seed=7)
    c, _ = E.WarehouseEnv("train").reset(seed=8)
    np.testing.assert_array_equal(a, b)
    assert not np.array_equal(a, c)


@pytest.mark.parametrize("seed", list(range(10)))
def test_generated_scene_is_valid(seed):
    e = E.WarehouseEnv("train")
    e.reset(seed=seed)
    sc = e.scenario
    assert sc.path.ndim == 2 and sc.path.shape[1] == 3
    assert E.path_length(sc.path) >= 2.0
    assert np.all(np.isfinite(sc.path)) and np.all(np.isfinite(sc.obstacles))
    assert np.max(np.hypot(*np.diff(sc.path[:, :2], axis=0).T)) <= 1.0  # ADR 0004 max_path_gap
    if len(sc.obstacles):
        # The global planner keeps the path free: no static point closer than ROBOT_RADIUS + 0.1 to it.
        d = np.min(np.hypot(*(sc.obstacles[:, None, :] - sc.path[None, :, :2]).transpose(2, 0, 1)))
        assert d >= spec.ROBOT_RADIUS + 0.1 - 1e-9
    assert math.hypot(sc.start[0] - sc.path[0, 0], sc.start[1] - sc.path[0, 1]) <= 0.2 * math.sqrt(2) + 1e-9


def test_randomization_ranges():
    for seed in range(20):
        e = E.WarehouseEnv("train", domain_randomization=True)
        _, info = e.reset(seed=seed)
        assert 0.8 <= info["gain_v"] <= 1.2 and 0.8 <= info["gain_w"] <= 1.2
        assert info["delay_steps"] in (0, 1, 2)


def test_no_randomization_outside_training_flag():
    e = E.WarehouseEnv("val", domain_randomization=False)
    _, info = e.reset(seed=1000)
    assert info["gain_v"] == 1.0 and info["gain_w"] == 1.0 and info["delay_steps"] == 0


# ---------------------------------------------------------------- dynamics


def test_command_limiter_from_rest():
    e, _, _ = nominal(scenario())
    obs, *_ = e.step(np.array([1.0, 1.0]))
    assert obs[59] * spec.V_MAX == pytest.approx(0.5 * spec.DT)  # max_accel 0.5 m/s^2
    assert obs[60] * spec.W_MAX == pytest.approx(2.0 * spec.DT)  # max_yaw_accel 2.0 rad/s^2


def test_nominal_kinematics_are_exact():
    e, _, _ = nominal(scenario())
    x = y = th = 0.0
    prev = (0.0, 0.0)
    for _ in range(30):
        e.step(np.array([1.0, 0.0 + 0.5]))  # targets v = 0.5, w = 0.5, reached through the limiter
        v, w = cc.limit_command((0.5, 0.5), prev, spec.DT)
        prev = (v, w)
        th1 = th + w * spec.DT
        x += v / w * (math.sin(th1) - math.sin(th))
        y -= v / w * (math.cos(th1) - math.cos(th))
        th = th1
    np.testing.assert_allclose(e.pose[:2], [x, y], atol=1e-9)


def test_delay_postpones_motion():
    e, _, _ = nominal(scenario())
    e.randomization = E.Randomization(delay_steps=2)
    e.pending = [np.zeros(2), np.zeros(2)]
    e.step(np.array([1.0, 0.0]))
    e.step(np.array([1.0, 0.0]))
    assert e.pose[0] == pytest.approx(0.0)  # the first command acts in the third period
    e.step(np.array([1.0, 0.0]))
    assert e.pose[0] > 0.0


# ---------------------------------------------------------------- reward and episode end


def test_driving_along_path_is_rewarded():
    e, _, _ = nominal(scenario())
    total = 0.0
    for _ in range(60):  # 3 s: 1 s ramp to V_MAX, then cruise; about 1.25 m
        _, r, terminated, truncated, info = e.step(np.array([1.0, 0.0]))
        total += r
        assert not terminated and not truncated
    assert info["progress"] > 1.0
    assert total > 0.0


def test_standing_still_is_not_rewarded():
    e, _, _ = nominal(scenario())
    total = sum(e.step(np.array([-1.0, 0.0]))[1] for _ in range(40))
    assert total < 0.0


def test_collision_terminates():
    sc = scenario(obstacles=[[0.6, y] for y in np.arange(-1.0, 1.01, 0.05)])  # wall across the path
    r, terminated, _, info = run(nominal(sc)[0], lambda e: np.array([1.0, 0.0]))
    assert terminated and info["collision"] and not info["success"]
    assert r <= E.R_COLLISION
    assert info["clearance"] < spec.ROBOT_RADIUS


def test_leaving_the_path_terminates():
    # Full speed with a constant turn: the robot circles away from a straight path.
    r, terminated, _, info = run(nominal(scenario(path=straight(8.0)))[0], lambda e: np.array([1.0, 1.0]))
    assert terminated and info["lost"] and not info["success"] and not info["collision"]
    assert abs(info["cross_track"]) > E.MAX_CROSS_TRACK
    assert r <= E.R_LOST + 1.0


def drive_then_brake(e):
    return np.array([1.0 if e.tracker.remaining_length() > 0.3 else -1.0, 0.0])


def test_reaching_goal_terminates_with_success():
    r, terminated, _, info = run(nominal(scenario(path=straight(0.8)))[0], drive_then_brake)
    assert terminated and info["success"] and not info["collision"]
    assert r >= E.R_GOAL - 1.0


def test_goal_needs_the_goal_yaw():
    # The goal yaw is +1 rad: stopping at the goal position is not enough, the robot must turn.
    e, _, _ = nominal(scenario(path=straight(0.8, goal_yaw=1.0)))
    for _ in range(100):
        _, _, terminated, _, info = e.step(drive_then_brake(e))
        assert not info["success"]
    turn = lambda env: np.array([-1.0, 0.4 if cc.normalize_angle(1.0 - env.pose[2]) > 0.05 else 0.0])
    r, terminated, _, info = run(e, turn)
    assert terminated and info["success"]


def test_turning_to_the_goal_yaw_is_rewarded():
    # Stopped near the goal with the goal yaw 1 rad to the left: turning towards it pays, standing does not.
    def at_goal():
        e, _, _ = nominal(scenario(path=straight(0.8, goal_yaw=1.0)))
        for _ in range(100):
            e.step(drive_then_brake(e))
        assert e.tracker.remaining_length() < E.GOAL_ZONE and e.measured[0] == 0.0
        return e

    # Half a second: the limiter ramps the yaw rate up, the first step alone does not pay yet.
    turn, stand = at_goal(), at_goal()
    turning = sum(turn.step(np.array([-1.0, 1.0]))[1] for _ in range(10))
    standing = sum(stand.step(np.array([-1.0, 0.0]))[1] for _ in range(10))
    assert turning > 0.0 > standing


def test_time_limit_truncates():
    e, _, _ = nominal(scenario(path=straight(2.0)))
    steps = 0
    while True:
        _, _, terminated, truncated, _ = e.step(np.array([-1.0, 0.0]))
        steps += 1
        if terminated or truncated:
            break
    assert truncated and not terminated
    assert steps == int((2.0 / E.TIME_LIMIT_SPEED + E.TIME_LIMIT_EXTRA) / spec.DT)


def test_observation_is_in_robot_frame():
    # Robot at (1, 1) facing +y; the path goes along +y from it: in base_link the path is along +x.
    path = np.array([[1.0, 1.0 + 0.1 * i, math.pi / 2] for i in range(31)])
    # The obstacle is 1.5 m ahead and 0.1 m to the left of the robot (away from a sector border).
    e, obs, _ = nominal(scenario(path=path, obstacles=[[0.9, 2.5]], start=(1.0, 1.0, math.pi / 2)))
    pts = obs[:20].reshape(10, 2) * spec.PATH_SCALE
    np.testing.assert_allclose(pts[0], [0.2, 0.0], atol=1e-6)
    assert obs[21] == pytest.approx(1.0)  # goal yaw equals the robot yaw: cos 1, sin 0
    ranges = obs[23:59] * spec.RANGE_MAX
    assert ranges[18] == pytest.approx(math.hypot(1.5, 0.1), abs=1e-6)
