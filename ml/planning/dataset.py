"""Generates the G3 learned-cost dataset (sim.py episodes) into data/g3/<split>/.

    python3 dataset.py [--out ../../data/g3] [--workers 4] [--split train]

Layouts of different splits never overlap (splits.json: layout seed ranges), so test layouts are
unseen in training (docs/EXPERIMENTS.md). Episode seed = layout seed * 1000 + episode index.
Each sample_NNNNN.npz holds `input` (float16, C x H x W), `label` and `length` (float16, H x W,
label NaN = no supervision), `static` (uint8 fine grid), `agents` (float32 T x A x 2), `t0_step`,
`observer`, `layout_seed`, `episode_seed`.
"""

import argparse
import json
import os
import subprocess
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import sim

HERE = os.path.dirname(os.path.abspath(__file__))


def jobs(splits, only=None):
    for split, spec in splits.items():
        if only and split != only:
            continue
        first, last = spec['layouts']
        for layout in range(first, last + 1):
            for k in range(spec['episodes_per_layout']):
                yield split, layout, layout * 1000 + k


def make(job):
    split, layout, seed, out = job
    path = os.path.join(out, split, f'sample_{seed:08d}.npz')
    if os.path.exists(path):
        return split, None
    e = sim.episode(seed, layout)
    np.savez_compressed(path, input=e['input'].astype(np.float16), label=e['label'].astype(np.float16),
                        length=e['length'].astype(np.float16), static=e['static'], agents=e['agents'],
                        t0_step=e['t0_step'], observer=e['observer'], layout_seed=layout, episode_seed=seed)
    labelled = np.isfinite(e['label'])
    return split, (int(labelled.sum()), float(np.nansum(e['label'])), e['arrived_fraction'])


def git_commit():
    try:
        commit = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, cwd=HERE).stdout.strip()
        dirty = bool(subprocess.run(['git', 'status', '--porcelain'], capture_output=True, text=True, cwd=HERE).stdout)
        return commit, dirty
    except OSError:
        return '', False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', default=os.path.join(HERE, '..', '..', 'data', 'g3'))
    parser.add_argument('--workers', type=int, default=os.cpu_count() or 1)
    parser.add_argument('--split', choices=['train', 'val', 'test'])
    args = parser.parse_args()
    with open(os.path.join(HERE, 'splits.json')) as f:
        splits = json.load(f)
    for split in splits:
        os.makedirs(os.path.join(args.out, split), exist_ok=True)
    todo = [(s, l, e, args.out) for s, l, e in jobs(splits, args.split)]
    stats = {}
    with ProcessPoolExecutor(args.workers) as pool:
        for split, result in pool.map(make, todo, chunksize=2):
            if result:
                n, total, arrived = result
                s = stats.setdefault(split, {'samples': 0, 'labelled_cells': 0, 'label_sum': 0.0, 'arrived': 0.0})
                s['samples'] += 1
                s['labelled_cells'] += n
                s['label_sum'] += total
                s['arrived'] += arrived
    commit, dirty = git_commit()
    meta = {'splits': splits, 'channels': sim.CHANNELS, 'resolution': sim.RESOLUTION, 'block': sim.BLOCK,
            'history_period': sim.HISTORY_PERIOD, 'window': sim.WINDOW, 'robot_speed': sim.ROBOT_SPEED,
            'stop_distance': sim.STOP_DISTANCE, 'max_label': sim.MAX_LABEL, 'git_commit': commit,
            'git_dirty': dirty, 'generated_now': stats}
    with open(os.path.join(args.out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    for split, s in stats.items():
        print(f"{split}: {s['samples']} samples, mean label {s['label_sum'] / max(s['labelled_cells'], 1):.3f}, "
              f"probes arrived {s['arrived'] / s['samples']:.2%}")


if __name__ == '__main__':
    main()
