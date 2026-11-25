"""Closed-loop comparison of the G3 methods on the unseen test layouts (data/g3/test).

    python3 evaluate.py --replay <path to planning_replay> [--models ../../data/g3/models]

For every test episode the observation at t0 (static map, observed occupancy t0..t0-3 s, observed
area) is exported and planned by the C++ planner (planning_replay: the same cost map, mask and A*
as planning_node) with method A (classical), method B (learned) and B without history (ablation).
Each path is then driven by the fixed classical controller of sim.run_probes among the *future*
agent motion. Metrics: planning success, arrival within the timeout, travel time and waiting time
(paired: queries all methods solved and completed), path length, planning time. Paired bootstrap
95 % intervals for the difference to method A over queries (5000 resamples, seed 12345).
"""

import argparse
import glob
import json
import os
import subprocess
import tempfile

import numpy as np

import sim

HERE = os.path.dirname(os.path.abspath(__file__))


def export_episode(z, directory, rng, queries=12):
    static = z['static']
    agents = z['agents']
    t0_step = int(z['t0_step'])
    observer = z['observer']
    shape = static.shape
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, 'meta.txt'), 'w') as f:
        f.write(f'{shape[1]} {shape[0]} {sim.RESOLUTION} {sim.ORIGIN[0]} {sim.ORIGIN[1]}\n')
    (static.astype(np.int8) * 100).tofile(os.path.join(directory, 'static.bin'))
    half = 0.5 * sim.WINDOW
    xs = sim.ORIGIN[0] + (np.arange(shape[1]) + 0.5) * sim.RESOLUTION
    ys = sim.ORIGIN[1] + (np.arange(shape[0]) + 0.5) * sim.RESOLUTION
    window = np.zeros(shape, bool)
    window[np.ix_(np.abs(ys - observer[1]) <= half, np.abs(xs - observer[0]) <= half)] = True
    window.astype(np.uint8).tofile(os.path.join(directory, 'area.bin'))
    for k in range(sim.HISTORY):
        step = max(t0_step - int(round(k * sim.HISTORY_PERIOD / sim.DT)), 0)
        occupied = sim.disc_mask(shape, agents[step], sim.AGENT_RADIUS) & window & (static == 0)
        occupied.astype(np.uint8).tofile(os.path.join(directory, f'observed_{k}.bin'))
    graph = sim.CoarseGraph(static, sim.ROBOT_RADIUS + 0.05)
    pairs = []
    now = agents[t0_step]
    while len(pairs) < queries:
        a, b = graph.centre(graph.random_free(rng)), graph.centre(graph.random_free(rng))
        if np.hypot(a[0] - b[0], a[1] - b[1]) < 4.0:
            continue
        if min(np.hypot(*(now - np.array(p)).T).min() for p in (a, b)) < 1.0:
            continue
        pairs.append((a, b))
    with open(os.path.join(directory, 'queries.txt'), 'w') as f:
        for a, b in pairs:
            f.write(f'{a[0]} {a[1]} {b[0]} {b[1]}\n')
    return pairs


def read_paths(path):
    out = []
    with open(path) as f:
        for line in f:
            v = line.split()
            status, cost, expanded, ms = int(v[0]), float(v[1]), int(v[2]), float(v[3])
            xy = np.array(v[4:], float).reshape(-1, 2)
            out.append({'ok': status == 0, 'cost': cost, 'expanded': expanded, 'ms': ms, 'xy': xy})
    return out


def resample(xy, step):
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.arange(0.0, s[-1] + 1e-9, step)
    if t[-1] < s[-1]:
        t = np.append(t, s[-1])
    return np.stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])], axis=1), s[-1]


def drive(paths, agents, t0_step):
    """Travel time, waiting time and arrival of each planned path under the fixed controller."""
    results = [None] * len(paths)
    todo = [i for i, p in enumerate(paths) if p['ok'] and len(p['xy']) >= 2]
    if not todo:
        return results
    polylines = [resample(paths[i]['xy'], sim.ROBOT_SPEED * sim.DT)[0] for i in todo]
    logs, arrived = sim.run_probes(None, polylines, agents, t0_step)
    for j, i in enumerate(todo):
        waited = sum(1 for _, _, w in logs[j] if w) * sim.DT
        results[i] = {'arrived': bool(arrived[j]), 'time': len(logs[j]) * sim.DT, 'wait': waited,
                      'length': resample(paths[i]['xy'], 1.0)[1]}
    return results


def bootstrap(diff, rng, n=5000):
    if len(diff) == 0:
        return (float('nan'),) * 3
    idx = rng.integers(len(diff), size=(n, len(diff)))
    means = diff[idx].mean(axis=1)
    return float(diff.mean()), float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--data', default=os.path.join(HERE, '..', '..', 'data', 'g3'))
    parser.add_argument('--models', default=None)
    parser.add_argument('--replay', required=True)
    parser.add_argument('--out', default=None)
    parser.add_argument('--queries', type=int, default=12)
    args = parser.parse_args()
    models = args.models or os.path.join(args.data, 'models')
    methods = {'A classical': ['classical'],
               'B learned': ['learned', os.path.join(models, 'latest', 'model.onnx')],
               'B without history': ['learned', os.path.join(models, 'no_history', 'model.onnx')]}
    rows = {m: [] for m in methods}
    rng = np.random.default_rng(12345)
    files = sorted(glob.glob(os.path.join(args.data, 'test', 'sample_*.npz')))
    with tempfile.TemporaryDirectory() as tmp:
        for n, path in enumerate(files):
            with np.load(path) as z:
                episode = {k: z[k] for k in z.files}
            directory = os.path.join(tmp, f'e{n}')
            export_episode(episode, directory, np.random.default_rng(int(episode['episode_seed'])), args.queries)
            for name, spec in methods.items():
                out = os.path.join(directory, name.replace(' ', '_') + '.txt')
                subprocess.run([args.replay, directory, out] + spec, check=True)
                paths = read_paths(out)
                driven = drive(paths, episode['agents'], int(episode['t0_step']))
                for q, (p, d) in enumerate(zip(paths, driven)):
                    rows[name].append({'episode': n, 'query': q, 'planned': p['ok'], 'ms': p['ms'],
                                       'expanded': p['expanded'], **(d or {})})
    names = list(methods)
    total = len(rows[names[0]])
    complete = [i for i in range(total) if all(rows[m][i].get('arrived') for m in names)]
    report = {'episodes': len(files), 'queries': total, 'paired_completed': len(complete), 'methods': {}}
    base = np.array([rows[names[0]][i]['time'] for i in complete])
    for name in names:
        r = rows[name]
        times = np.array([r[i]['time'] for i in complete])
        waits = np.array([r[i]['wait'] for i in complete])
        lengths = np.array([r[i]['length'] for i in complete])
        ms = np.array([x['ms'] for x in r if x['planned']])
        report['methods'][name] = {
            'planning_success': float(np.mean([x['planned'] for x in r])),
            'arrival_rate': float(np.mean([bool(x.get('arrived')) for x in r])),
            'travel_time_s': float(times.mean()), 'wait_time_s': float(waits.mean()),
            'path_length_m': float(lengths.mean()),
            'planning_ms_p50': float(np.percentile(ms, 50)), 'planning_ms_p95': float(np.percentile(ms, 95)),
            'expanded_mean': float(np.mean([x['expanded'] for x in r if x['planned']])),
            'travel_time_diff_to_A_s': bootstrap(times - base, rng),
            'wait_time_diff_to_A_s': bootstrap(waits - np.array([rows[names[0]][i]['wait'] for i in complete]), rng),
        }
    text = json.dumps(report, indent=2)
    print(text)
    out = args.out or os.path.join(models, 'evaluation.json')
    with open(out, 'w') as f:
        f.write(text + '\n')


if __name__ == '__main__':
    main()
