"""Export of a trained PPO policy into the capstone_control package: ONNX model, manifest, golden data.

    python3 export_onnx.py ../../data/g4/models/seed_1/best_model.zip

Writes
  ros2_ws/src/capstone_control/models/ppo_policy.onnx   float [1, OBS_SIZE] -> float [1, ACTION_SIZE]
  ros2_ws/src/capstone_control/models/ppo_policy.json   manifest: SHA-256 of the model and of spec.py, the
                                                        training manifest, ONNX vs PyTorch difference
  ros2_ws/src/capstone_control/test/data/ppo_golden.txt observations of val scenes and the ONNX Runtime
                                                        outputs; test_ppo_controller.cpp must reproduce them

The exported network is the deterministic policy: the mean of the Gaussian, clipped to [-1, 1] as
Stable-Baselines3 clips actions to the action space; spec.action_to_target and the CommandLimiter come after it
in C++ (PpoController, the node), not inside the network. Which training seed is packaged is decided by validation
only; the report shows all three seeds.
"""

import argparse
import hashlib
import importlib.metadata
import json
import pathlib

import numpy as np
import onnx
import onnxruntime
import torch
from stable_baselines3 import PPO

import env as E
import spec

PACKAGE = pathlib.Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "capstone_control"
GOLDEN_CASES = 50


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


class DeterministicActor(torch.nn.Module):
    """torch.nn.Module: observation -> clip(mean action, -1, 1) of an SB3 ActorCriticPolicy."""

    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs):
        features = self.policy.extract_features(obs, self.policy.pi_features_extractor)
        latent_pi = self.policy.mlp_extractor.forward_actor(features)
        return torch.clamp(self.policy.action_net(latent_pi), -1.0, 1.0)


def golden_observations(count):
    """Observations from val scenes along a short rollout of a simple controller (not from the test split)."""
    env = E.WarehouseEnv("val", domain_randomization=False)
    rng = np.random.default_rng(12345)
    obs_list = []

    val_seeds = sorted(E.SPLITS["val"])
    pick_indices = np.linspace(0, len(val_seeds) - 1, 12, dtype=int)
    seed_picks = [val_seeds[i] for i in pick_indices]

    # Phase 1: rollouts with mixed random/straight actions on val scenes: exercises path points,
    # obstacle sectors and velocity. Straight stretches take the robot past static boxes.
    for seed in seed_picks:
        if len(obs_list) >= count // 2:
            break
        obs, _ = env.reset(seed=seed, options={"randomize": False})
        obs_list.append(obs.copy())
        for k in range(80):
            if len(obs_list) >= count // 2:
                break
            if k % 5 == 0:
                action = np.array([0.5, 0.0], dtype=np.float32)  # straight ahead
            else:
                action = rng.uniform(-0.6, 0.6, size=spec.ACTION_SIZE).astype(np.float32)
            obs, _r, term, trunc, _ = env.step(action)
            obs_list.append(obs.copy())
            if term or trunc:
                break

    # Phase 2: synthetic "at the goal" observations (small remaining, small goal-yaw error, low velocity).
    for seed in seed_picks:
        if len(obs_list) >= count:
            break
        env.reset(seed=seed, options={"randomize": False})
        goal = env.tracker.goal()
        for dx, dy, dyaw, v, w in [
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (-0.15, 0.05, 0.1, 0.01, 0.03),
            (0.1, -0.1, -0.15, 0.02, -0.02),
            (-0.2, 0.0, 0.2, 0.0, 0.0),
        ]:
            if len(obs_list) >= count:
                break
            env.pose = np.array([goal[0] + dx, goal[1] + dy, goal[2] + dyaw])
            env.tracker.update(env.pose)
            env.velocity = np.array([v, w])
            env.measured = env.velocity.copy()
            env.command = np.zeros(2)
            obs_list.append(env._observe().copy())

    all_obs = np.stack(obs_list).astype(np.float32)
    if len(all_obs) < count:
        raise RuntimeError(f"collected {len(all_obs)} golden observations, need {count}")
    perm = np.random.default_rng(999).permutation(len(all_obs))
    return all_obs[perm[:count]]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checkpoint", help="best_model.zip of a training run")
    args = parser.parse_args()

    ckpt = pathlib.Path(args.checkpoint).resolve()
    model = PPO.load(str(ckpt), device="cpu")
    model.policy.eval()
    actor = DeterministicActor(model.policy).eval()

    # Sanity check: the wrapper must equal model.predict(deterministic=True) on a few observations.
    rng = np.random.default_rng(7)
    probe = rng.uniform(-1.0, 1.0, size=(5, spec.OBS_SIZE)).astype(np.float32)
    with torch.no_grad():
        probe_torch = actor(torch.from_numpy(probe)).cpu().numpy()
    for i in range(len(probe)):
        sb3_action, _ = model.predict(probe[i], deterministic=True)
        diff = float(np.max(np.abs(sb3_action - probe_torch[i])))
        if diff > 1e-5:
            raise RuntimeError(f"DeterministicActor vs predict mismatch: {diff} on probe {i}")

    onnx_path = PACKAGE / "models" / "ppo_policy.onnx"
    onnx_path.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.zeros(1, spec.OBS_SIZE, dtype=torch.float32)
    with torch.no_grad():
        out_shape = tuple(actor(dummy).shape)
    if out_shape != (1, spec.ACTION_SIZE):
        raise RuntimeError(f"unexpected output shape {out_shape}")
    torch.onnx.export(
        actor, dummy, str(onnx_path),
        input_names=["observation"], output_names=["action"],
        opset_version=17, do_constant_folding=True,
    )
    onnx.checker.check_model(onnx.load(str(onnx_path)))

    obs = golden_observations(GOLDEN_CASES)
    with torch.no_grad():
        torch_out = actor(torch.from_numpy(obs)).cpu().numpy()

    sess = onnxruntime.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    onnx_out = np.stack([
        sess.run(None, {"observation": obs[i:i + 1]})[0][0] for i in range(len(obs))
    ])
    if onnx_out.shape != torch_out.shape or onnx_out.shape != (GOLDEN_CASES, spec.ACTION_SIZE):
        raise RuntimeError(f"ONNX output shape {onnx_out.shape} != torch {torch_out.shape}")
    max_diff = float(np.max(np.abs(onnx_out - torch_out)))
    if max_diff >= 1e-5:
        raise RuntimeError(f"ONNX vs PyTorch diff {max_diff} >= 1e-5")

    golden_path = PACKAGE / "test" / "data" / "ppo_golden.txt"
    golden_path.parent.mkdir(parents=True, exist_ok=True)
    with golden_path.open("w") as f:
        f.write(
            f"# {GOLDEN_CASES} cases: {spec.OBS_SIZE} obs + {spec.ACTION_SIZE} action "
            f"per line; outputs from ONNX Runtime\n"
        )
        for i in range(len(obs)):
            vals = list(map(float, obs[i])) + list(map(float, onnx_out[i]))
            f.write(" ".join(repr(v) for v in vals) + "\n")

    train_manifest = json.loads((ckpt.parent / "manifest.json").read_text())
    manifest = {
        "model_sha256": sha256(onnx_path),
        "spec_sha256": sha256(pathlib.Path(spec.__file__)),
        "input": {"name": "observation", "shape": [1, spec.OBS_SIZE], "dtype": "float32"},
        "output": {"name": "action", "shape": [1, spec.ACTION_SIZE], "dtype": "float32"},
        "training_manifest": train_manifest,
        "onnx_vs_torch_max_abs_diff": max_diff,
        "versions": {
            "torch": torch.__version__,
            "onnx": importlib.metadata.version("onnx"),
            "onnxruntime": importlib.metadata.version("onnxruntime"),
        },
    }
    (PACKAGE / "models" / "ppo_policy.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )


if __name__ == "__main__":
    main()
