"""Training of the G4 learned controller (method B): PPO from Stable-Baselines3 on WarehouseEnv.

    python3 train.py --seed 0                      # one training seed, domain randomization on
    python3 train.py --seed 0 --no-randomization   # ablation: PPO without domain randomization
    for s in 0 1 2; do python3 train.py --seed $s; done

Only the "train" split is used for updates and only the "val" split for model selection
(docs/EXPERIMENTS.md); the test split is never read here. The best checkpoint by validation success rate is
kept with a manifest that makes the run reproducible.

Output: <out>/seed_<seed>[_no_dr]/ with best_model.zip, last_model.zip, manifest.json, progress.csv.
"""

import argparse
import importlib.metadata
import json
import os
import pathlib
import subprocess
import time

import gymnasium as gym
import numpy as np

import env as E
import spec

# PPO hyperparameters, fixed in Git (projects/04_control.md: training budget is part of the method).
TOTAL_TIMESTEPS = 3_000_000
N_ENVS = 8
PPO_KWARGS = dict(
    n_steps=1024,  # per environment: 8 * 1024 transitions per update
    batch_size=512,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.0,
    learning_rate=3e-4,
    policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]), log_std_init=-0.5),
)
EVAL_EVERY = 100_000  # timesteps
EVAL_EPISODES = 100  # the whole val split
TORCH_THREADS = 1  # a 2 x 128 MLP: more threads only add synchronisation overhead


class SplitSampler(gym.Wrapper):
    """gymnasium wrapper: every reset draws the next scene seed of the split from its own generator, so the
    sequence of scenes depends only on the training seed and the environment index."""

    def __init__(self, env, seed):
        super().__init__(env)
        self.rng = np.random.default_rng(seed)
        self.seeds = np.array(E.SPLITS[env.unwrapped.split])
        self.episodes = 0

    def reset(self, *, seed=None, options=None):
        # SB3 passes its own seed to the first reset; the scene seed must come from the split instead.
        del seed
        self.episodes += 1
        return self.env.reset(seed=int(self.rng.choice(self.seeds)), options=options)


def make_env(split, randomization, training_seed, index):
    """Factory for SubprocVecEnv / DummyVecEnv: one WarehouseEnv in a SplitSampler and a Monitor."""

    def build():
        from stable_baselines3.common.monitor import Monitor

        env = E.WarehouseEnv(split, domain_randomization=randomization)
        # Monitor outside: it sees the episode returns and lengths, SB3 logs them as rollout/ep_rew_mean.
        return Monitor(SplitSampler(env, seed=[training_seed, index]))

    return build


def evaluate(model, split="val", episodes=EVAL_EPISODES, randomization=False):
    """Deterministic policy on the first `episodes` seeds of a split; returns success rate, collision rate,
    lost rate (left the path), timeout rate, mean return, mean cross-track RMSE and mean min clearance (capped
    at spec.RANGE_MAX: a scene without obstacles near the path would give infinity). Success is the FollowPath
    success of ADR 0004 (info["success"]), the same as in the robot."""
    env = E.WarehouseEnv(split, domain_randomization=randomization)
    rows = []
    for seed in sorted(E.SPLITS[split])[:episodes]:
        obs, info = env.reset(seed=seed)
        total, cross, clearance = 0.0, [], [min(info["clearance"], spec.RANGE_MAX)]
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            cross.append(info["cross_track"])
            clearance.append(min(info["clearance"], spec.RANGE_MAX))
            if terminated or truncated:
                break
        # Per episode first, then over episodes: long episodes must not weigh more (docs/EXPERIMENTS.md).
        rows.append((info["success"], info["collision"], info["lost"], truncated, total,
                     float(np.sqrt(np.mean(np.square(cross)))), float(np.min(clearance))))
    success, collision, lost, timeout, ret, rmse, clear = (np.array(c, dtype=float) for c in zip(*rows))
    return {
        "success_rate": float(success.mean()),
        "collision_rate": float(collision.mean()),
        "lost_rate": float(lost.mean()),
        "timeout_rate": float(timeout.mean()),
        "mean_return": float(ret.mean()),
        "cross_track_rmse": float(rmse.mean()),
        "min_clearance": float(clear.mean()),
        "episodes": len(rows),
    }


def selection_key(metrics):
    """Model selection on val: success first, then fewer collisions, then return."""
    return (metrics["success_rate"], -metrics["collision_rate"], metrics["mean_return"])


def git_commit():
    """(commit, dirty) of the working tree; GIT_COMMIT / GIT_DIRTY when git is not in the container."""
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        status = subprocess.check_output(["git", "status", "--porcelain"], text=True, stderr=subprocess.DEVNULL)
        return commit, bool(status.strip())
    except (OSError, subprocess.CalledProcessError):
        dirty = os.environ.get("GIT_DIRTY")
        return os.environ.get("GIT_COMMIT"), None if dirty is None else dirty == "true"


def versions():
    packages = ["stable-baselines3", "torch", "gymnasium", "numpy"]
    return {p: importlib.metadata.version(p) for p in packages}


def constants(module, names):
    return {n: getattr(module, n) for n in names}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, required=True, help="training seed (0, 1, 2 for the report)")
    parser.add_argument("--timesteps", type=int, default=TOTAL_TIMESTEPS)
    parser.add_argument("--envs", type=int, default=N_ENVS)
    parser.add_argument("--no-randomization", action="store_true", help="ablation without domain randomization")
    parser.add_argument("--out", default="../../data/g4/models")  # data/ is not in Git
    args = parser.parse_args()

    # Heavy imports after argparse: --help stays fast.
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.utils import set_random_seed
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

    name = f"seed_{args.seed}" + ("_no_dr" if args.no_randomization else "")
    out = pathlib.Path(args.out) / name
    out.mkdir(parents=True, exist_ok=True)

    set_random_seed(args.seed)  # python, numpy and torch
    torch.set_num_threads(TORCH_THREADS)
    factories = [make_env("train", not args.no_randomization, args.seed, i) for i in range(args.envs)]
    vec_env = SubprocVecEnv(factories) if args.envs > 1 else DummyVecEnv(factories)

    model = PPO("MlpPolicy", vec_env, seed=args.seed, verbose=1, device="cpu", **PPO_KWARGS)
    model.set_logger(configure(str(out), ["csv", "stdout"]))

    commit, dirty = git_commit()
    manifest = {
        "name": name,
        "arguments": vars(args),
        "ppo": PPO_KWARGS,
        "eval_every": EVAL_EVERY,
        "spec": constants(spec, ["OBS_SIZE", "ACTION_SIZE", "V_MAX", "W_MAX", "DT", "ROBOT_RADIUS", "PATH_POINTS",
                                 "PATH_SPACING", "PATH_SCALE", "REMAINING_MAX", "SECTORS", "RANGE_MAX"]),
        "env": constants(E, ["W_PROGRESS", "W_TRACK", "W_SMOOTH", "W_PROXIMITY", "W_GOAL_DISTANCE",
                             "W_GOAL_HEADING", "W_TIME",
                             "D_SAFE", "GOAL_ZONE", "R_COLLISION", "R_GOAL", "MAX_CROSS_TRACK", "R_LOST",
                             "GOAL_XY", "GOAL_YAW", "GOAL_SPEED", "GOAL_YAW_RATE", "TRACKER_WINDOW",
                             "TIME_LIMIT_SPEED", "TIME_LIMIT_EXTRA"]),
        "splits": {k: [v.start, v.stop] for k, v in E.SPLITS.items()},
        "selection": "val success rate, then lower collision rate, then mean return; deterministic policy, "
                     "nominal robot",
        "git_commit": commit,
        "git_dirty": dirty,
        "versions": versions(),
        "evaluations": [],
        "best": None,
    }

    def write_manifest():
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    start = time.time()
    best = None
    while model.num_timesteps < args.timesteps:
        model.learn(min(EVAL_EVERY, args.timesteps - model.num_timesteps), reset_num_timesteps=False)
        metrics = evaluate(model, "val")
        metrics["timesteps"] = model.num_timesteps
        metrics["seconds"] = round(time.time() - start, 1)
        manifest["evaluations"].append(metrics)
        for key, value in metrics.items():
            model.logger.record(f"val/{key}", value)
        model.logger.dump(model.num_timesteps)
        if best is None or selection_key(metrics) > selection_key(best):
            best = metrics
            model.save(out / "best_model.zip")
            manifest["best"] = metrics
        write_manifest()  # after every evaluation: an interrupted run still documents itself

    model.save(out / "last_model.zip")
    manifest["training_seconds"] = round(time.time() - start, 1)
    write_manifest()
    vec_env.close()
    print(f"best on val at {best['timesteps']} steps: success {best['success_rate']:.2f}, "
          f"collisions {best['collision_rate']:.2f} -> {out / 'best_model.zip'}")


if __name__ == "__main__":
    main()
