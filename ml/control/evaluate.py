"""Isolated evaluation of trained policies on the test and OOD splits (docs/EXPERIMENTS.md, component level).

    python3 evaluate.py ../../data/g4/models/seed_0/best_model.zip ../../data/g4/models/seed_1/best_model.zip ...
    python3 evaluate.py --split ood --mismatch 0.8 ...   # payload / model mismatch: both gains fixed to 0.8

Writes one CSV row per episode (episode is the unit of analysis): model, training seed, split, scene seed,
success, collision, timeout, time, path length driven, cross-track RMSE and p95, mean |yaw error|, min clearance,
control variation sum ||u[t] - u[t-1]||, constraint violations (limited commands outside the ADR 0004
boxes: must be 0, the limiter is in the loop). Success is the ADR 0004 FollowPath success (info["success"]). Aggregates and confidence intervals are
computed from this CSV in the report, not here. The MPC is compared on the same scenes in Webots replay
(capstone_benchmark); this script only covers the learned policy in the training environment.
"""

import argparse
import csv
import json
import pathlib

import numpy as np
from stable_baselines3 import PPO

import control_core as cc
import env as E
import spec

FIELDS = [
    "model", "train_seed", "ablation", "split", "scene_seed",
    "success", "collision", "timeout", "time", "path_length",
    "cross_track_rmse", "cross_track_p95", "yaw_err_mean_abs",
    "min_clearance", "control_variation", "constraint_violations",
]

TOL = 1e-6  # FP slack for the limiter checks; the limiter itself is exact but clip/rounding can wobble


def run_episode(model, environment, seed, options=None, mismatch=None):
    """One deterministic episode; returns the CSV row as a dict."""
    obs, info = environment.reset(seed=seed, options=options or {"randomize": False})
    if mismatch is not None:
        # Randomization() is nominal after reset(randomize=False); only the gains change.
        environment.randomization.gain_v = float(mismatch)
        environment.randomization.gain_w = float(mismatch)

    lim = environment.limits
    max_dw = lim.max_yaw_accel * spec.DT
    dv_accel = lim.max_accel * spec.DT
    dv_decel = lim.max_decel * spec.DT
    v_reverse = lim.max_reverse_speed

    last_xy = environment.pose[:2].copy()
    prev_cmd = np.zeros(2)
    commands = []
    cross_track = []
    yaw_err = []
    clearance = []
    path_length = 0.0
    steps = 0
    violations = 0
    truncated = False

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, _reward, terminated, truncated, info = environment.step(action)
        steps += 1

        cmd = environment.command.copy()
        commands.append(cmd)
        # ADR 0004 boxes: |v| <= V_MAX, |w| <= W_MAX, |dw| <= max_yaw_accel * DT,
        # |dv| bounded by max_accel when speeding up and max_decel when slowing down,
        # and reverse is bounded by max_reverse_speed.
        if abs(cmd[0]) > spec.V_MAX + TOL: violations += 1
        if cmd[0] < -v_reverse - TOL: violations += 1
        if abs(cmd[1]) > spec.W_MAX + TOL: violations += 1
        dv = cmd[0] - prev_cmd[0]
        if abs(dv) > (dv_accel if dv >= 0 else dv_decel) + TOL: violations += 1
        if abs(cmd[1] - prev_cmd[1]) > max_dw + TOL: violations += 1
        prev_cmd = cmd

        xy = environment.pose[:2]
        path_length += float(np.hypot(*(xy - last_xy)))
        last_xy = xy.copy()

        cross_track.append(float(info["cross_track"]))
        clearance.append(min(float(info["clearance"]), spec.RANGE_MAX))
        goal = environment.tracker.goal()
        yaw_err.append(abs(float(cc.normalize_angle(goal[2] - environment.pose[2]))))

        if terminated or truncated:
            break

    cmds = np.asarray(commands, dtype=np.float64)
    if len(cmds) >= 2:
        control_variation = float(np.sum(np.linalg.norm(np.diff(cmds, axis=0), axis=1)))
    else:
        control_variation = 0.0

    ct = np.asarray(cross_track, dtype=np.float64)
    return {
        "success": int(bool(info["success"])),
        "collision": int(bool(info["collision"])),
        "timeout": int(bool(truncated)),
        "time": float(steps * spec.DT),
        "path_length": float(path_length),
        "cross_track_rmse": float(np.sqrt(np.mean(ct ** 2))),
        "cross_track_p95": float(np.percentile(ct, 95)),
        "yaw_err_mean_abs": float(np.mean(yaw_err)),
        "min_clearance": float(np.min(clearance)),
        "control_variation": control_variation,
        "constraint_violations": int(violations),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checkpoints", nargs="+")
    parser.add_argument("--split", default="test", choices=["test", "ood"])
    parser.add_argument("--mismatch", type=float, default=None, help="fix both actuation gains to this value")
    parser.add_argument("--out", default="../../data/g4/eval.csv")
    args = parser.parse_args()
    
    seeds = sorted(E.SPLITS[args.split])  # all of them, fixed before evaluation
    environment = E.WarehouseEnv(args.split, domain_randomization=False)

    rows = []
    for ckpt_str in args.checkpoints:
        ckpt = pathlib.Path(ckpt_str).resolve()
        manifest = json.loads((ckpt.parent / "manifest.json").read_text())
        train_seed = int(manifest["arguments"]["seed"])
        ablation = int(bool(manifest["arguments"]["no_randomization"]))
        model = PPO.load(str(ckpt), device="cpu")

        for scene_seed in seeds:
            row = run_episode(model, environment, scene_seed,
                              options={"randomize": False}, mismatch=args.mismatch)
            row.update(
                model=manifest["name"],
                train_seed=train_seed,
                ablation=ablation,
                split=args.split,
                scene_seed=scene_seed,
            )
            rows.append(row)

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
