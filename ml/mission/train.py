"""Обучение трёх seeds; выбор checkpoint только по validation, без test tuning."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import time

from dqn import Network, ReplayBuffer
from env import DispatchEnvironment, FAMILIES, make_scenario
import numpy as np
from spec import masked_argmax


def validate(network, congestion_feature):
    returns = []
    for family in FAMILIES:
        for seed in range(1000, 1010):
            scenario = make_scenario('validation', seed, family)
            env = DispatchEnvironment(scenario, congestion_feature)
            inputs, mask = env.observe()
            total = 0.0
            while not env.done:
                action = masked_argmax(network.predict(inputs), mask)
                inputs, mask, reward, _, _ = env.step(action)
                total += reward
            returns.append(total)
    return float(np.mean(returns))


def train_seed(seed, episodes, output, congestion_feature):
    rng = np.random.default_rng(seed)
    network = Network(rng)
    target = Network(rng)
    target.copy_from(network)
    replay = ReplayBuffer()
    rows = []
    best_score = -np.inf
    steps = 0
    start = time.monotonic()
    output.mkdir(parents=True, exist_ok=True)
    for episode in range(episodes):
        family = FAMILIES[episode % len(FAMILIES)]
        scenario_seed = int(rng.integers(0, 1000))
        scenario = make_scenario('train', scenario_seed, family)
        env = DispatchEnvironment(scenario, congestion_feature)
        inputs, mask = env.observe()
        total = 0.0
        losses = []
        epsilon = max(0.05, 1.0 - episode / max(1.0, episodes * 0.75))
        while not env.done:
            if rng.random() < epsilon:
                action = int(rng.choice(np.flatnonzero(mask)))
            else:
                action = masked_argmax(network.predict(inputs), mask)
            next_inputs, next_mask, reward, done, duration = env.step(action)
            replay.append(inputs, action, reward, next_inputs, next_mask, done, duration)
            if replay.size >= 128 and steps % 2 == 0:
                losses.append(replay.train_batch(network, target, rng))
            if steps % 250 == 0:
                target.copy_from(network)
            steps += 1
            inputs, mask = next_inputs, next_mask
            total += reward
        row = {'episode': episode + 1, 'scenario_seed': scenario_seed, 'family': family,
               'return': total, 'epsilon': epsilon,
               'loss': float(np.mean(losses)) if losses else 0.0, 'validation_return': ''}
        if (episode + 1) % 100 == 0 or episode == episodes - 1:
            score = validate(network, congestion_feature)
            row['validation_return'] = score
            if score > best_score:
                best_score = score
                network.export(output / 'model.json', {
                    'training_seed': seed, 'episode': episode + 1,
                    'validation_return': score, 'congestion_feature': congestion_feature,
                })
            print(f'seed={seed} congestion={congestion_feature} '
                  f'episode={episode + 1}/{episodes} validation={score:.3f}', flush=True)
        rows.append(row)
    with (output / 'learning.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return {'training_seed': seed, 'episodes': episodes, 'transitions': steps,
            'optimizer_updates': network.updates, 'validation_return': best_score,
            'training_seconds': time.monotonic() - start,
            'congestion_feature': congestion_feature,
            'sha256': hashlib.sha256((output / 'model.json').read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=1000)
    parser.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    parser.add_argument('--out', type=Path, default=Path('data/g5/training'))
    parser.add_argument('--no-congestion', action='store_true')
    args = parser.parse_args()
    if args.episodes < 1 or not args.seeds:
        parser.error('positive episodes and at least one seed required')
    reports = [train_seed(seed, args.episodes, args.out / f'seed_{seed}',
                          not args.no_congestion) for seed in args.seeds]
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip())
    metadata = {'algorithm': 'DQN with target network, replay, Huber loss and Adam',
                'numpy_version': np.__version__, 'git_commit': commit, 'git_dirty': dirty,
                'episodes': args.episodes, 'seeds': reports,
                'selection': 'maximum mean validation return over fixed 50 validation episodes'}
    (args.out / 'training.json').write_text(json.dumps(metadata, indent=2) + '\n')


if __name__ == '__main__':
    main()
