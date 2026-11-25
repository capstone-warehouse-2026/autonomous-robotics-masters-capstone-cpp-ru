#!/usr/bin/env python3
"""Offline G1 training/evaluation. GT is only read here, never by C++ localization."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import numpy as np

BASE = np.array([.04**2, .04**2, .025**2])
LOW = np.array([1e-5]*3)
HIGH = np.array([1., 1., .25])


def wrap(a):
    return (a+np.pi) % (2*np.pi)-np.pi


def load_dataset(path):
    path = Path(path)
    m = json.loads((path/'manifest.json').read_text())
    if m['schema'] != 1 or m['source'] != 'recorded_rosbag':
        raise ValueError('Only explicitly recorded_rosbag datasets accepted by the experiment pipeline')
    if not m.get('world_sha256') or set(m['sha256']) != {'map.txt', 'events.txt', 'ground_truth.csv'}:
        raise ValueError('Required provenance/hash fields missing')
    for name, expected in m['sha256'].items():
        if Path(name).name != name:
            raise ValueError('Invalid manifest filename')
        if hashlib.sha256((path/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Hash mismatch: {path/name}')
    return path, m


def validate_splits(items):
    worlds, trajectories, world_hashes, event_hashes = {}, {}, {}, {}
    for _, m in items:
        for table, key in [(worlds, m['world']), (trajectories, m['trajectory'])]:
            if key in table and table[key] != m['split']:
                raise ValueError(f'Split leakage for {key}')
            table[key] = m['split']
        for table, key in [(world_hashes, m.get('world_sha256')), (event_hashes, m.get('sha256', {}).get('events.txt'))]:
            if key is None: continue
            if key in table and table[key] != m['split']:
                raise ValueError('Identical world/event content appears across splits')
            table[key] = m['split']


def replay(exe, dataset, output, method='classical', model='-', scale=1.):
    path, m = dataset
    subprocess.run([str(exe), str(path/'map.txt'), str(path/'events.txt'), str(output),
                    method, str(model), *map(str, m['initial']), str(scale)], check=True)
    return np.atleast_1d(np.genfromtxt(output, names=True, delimiter=',', dtype=None, encoding='utf-8'))


def truth_at(path, times):
    gt = np.atleast_1d(np.genfromtxt(path/'ground_truth.csv', names=True, delimiter=','))
    t = gt['stamp']
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('GT must have strictly increasing times')
    idx = np.searchsorted(t, times).clip(1, len(t)-1)
    valid = (times >= t[0]) & (times <= t[-1]) & ((t[idx]-t[idx-1]) <= .2)
    poses = np.column_stack([np.interp(times, t, gt['x']), np.interp(times, t, gt['y']),
                             wrap(np.interp(times, t, np.unwrap(gt['yaw'])))])
    return poses, valid


def labels(dataset, rows):
    scans = rows[(rows['kind'] == 'S') & np.isfinite(rows['mx'])]
    gt, valid = truth_at(dataset[0], scans['stamp'])
    scans, gt = scans[valid], gt[valid]
    x = np.column_stack([scans[f'f{i}'] for i in range(7)])
    error = np.column_stack([scans['mx'], scans['my'], scans['myaw']])-gt
    error[:, 2] = wrap(error[:, 2])
    finite = np.all(np.isfinite(x), axis=1) & np.all(np.isfinite(error), axis=1)
    return x[finite], error[finite]**2


def forward(x, params):
    w1, b1, w2, b2 = params
    hidden = np.tanh(x@w1+b1)
    z = hidden@w2+b2
    raw = np.logaddexp(0, z)
    variance = np.clip(raw, LOW, HIGH)
    return variance, hidden, z, raw


def loss(x, targets, params):
    v, _, _, _ = forward(x, params)
    return float(.5*np.mean(np.log(v)+targets/v))


def fit(x, y, vx, vy, epochs=500, seed=7, no_history=False):
    if len(x) < 20 or len(vx) < 10:
        raise ValueError('Need at least 20 training and 10 validation scan matches')
    mean, scale = x.mean(0), np.maximum(x.std(0), 1e-4)
    x, vx = (x-mean)/scale, (vx-mean)/scale
    if no_history:
        x[:, 6] = 0; vx[:, 6] = 0
    rng = np.random.default_rng(seed)
    params = [rng.normal(0, .1, (7, 8)), np.zeros(8), rng.normal(0, .01, (8, 3)), np.log(np.expm1(BASE))]
    if no_history:
        params[0][6] = 0
    first = [np.zeros_like(p) for p in params]
    second = [np.zeros_like(p) for p in params]
    best, best_loss, best_epoch = None, float('inf'), 0
    for epoch in range(1, epochs+1):
        variance, h, z, raw = forward(x, params)
        dz = .5*(1/variance-y/variance**2)/(len(x)*3)
        dz *= 1/(1+np.exp(-np.clip(z, -700, 700)))
        dz *= (raw > LOW) & (raw < HIGH)
        dh = (dz@params[2].T)*(1-h*h)
        grads = [x.T@dh, dh.sum(0), h.T@dz, dz.sum(0)]
        if no_history:
            grads[0][6] = 0
        for i, g in enumerate(grads):
            g = np.clip(g, -10, 10)
            first[i] = .9*first[i]+.1*g; second[i] = .999*second[i]+.001*g*g
            params[i] -= .01*(first[i]/(1-.9**epoch))/(np.sqrt(second[i]/(1-.999**epoch))+1e-8)
        current = loss(vx, vy, params)
        if np.isfinite(current) and current < best_loss:
            best_loss, best_epoch = current, epoch
            best = [p.copy() for p in params]
    if best is None:
        raise ValueError('Training produced no finite validation loss')
    # Absorb preprocessing into first-layer weights for exact C++ raw-feature inference.
    best[0] /= scale[:, None]
    best[1] -= mean@best[0]
    return best, dict(validation_nll=best_loss, selected_epoch=best_epoch, seed=seed,
                      train_matches=len(x), validation_matches=len(vx), no_history=no_history)


def export_model(path, params):
    w1, b1, w2, b2 = params
    weights = np.concatenate([np.column_stack([w1.T, b1]).ravel(),
                              np.column_stack([w2.T, b2]).ravel()])
    if len(weights) != 91 or not np.all(np.isfinite(weights)):
        raise ValueError('Invalid export')
    path.write_text('G1_MLP_V1\n'+'\n'.join(format(v, '.17g') for v in weights)+'\n')


def train(args):
    items = [load_dataset(p) for p in args.datasets]
    validate_splits(items)
    if any(m['split'] == 'test' for _, m in items):
        raise ValueError('Test data forbidden in training/model selection')
    args.output.mkdir(parents=True, exist_ok=False)
    groups = {'train': [], 'validation': []}
    for i, item in enumerate(items):
        rows = replay(args.replay, item, args.output/f'calibration_{i}.csv')
        groups[item[1]['split']].append(labels(item, rows))
    if not all(groups.values()):
        raise ValueError('Separate train and validation worlds/trajectories required')
    x, y = [np.concatenate([r[k] for r in groups['train']]) for k in (0, 1)]
    vx, vy = [np.concatenate([r[k] for r in groups['validation']]) for k in (0, 1)]
    scale = float(np.clip(np.mean(vy/BASE), .01, 400))
    params, info = fit(x, y, vx, vy, args.epochs, args.seed, args.no_history)
    export_model(args.output/'model.txt', params)
    info.update(classical_scale=scale, datasets=[m for _, m in items],
                model_sha256=hashlib.sha256((args.output/'model.txt').read_bytes()).hexdigest())
    (args.output/'training.json').write_text(json.dumps(info, indent=2)+'\n')
    print(json.dumps({k: v for k, v in info.items() if k != 'datasets'}, indent=2))


def relative(a, b):
    d = b-a; c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    return np.column_stack([c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1], wrap(d[:, 2])])


def metrics(dataset, rows):
    predictions = rows[rows['kind'] == 'W']
    gt, valid = truth_at(dataset[0], predictions['stamp'])
    predictions, gt = predictions[valid], gt[valid]
    if len(predictions) < 2:
        raise ValueError('Too few synchronized estimates')
    t = predictions['stamp']; estimated = np.column_stack([predictions[k] for k in ('x', 'y', 'yaw')])
    error = estimated-gt; error[:, 2] = wrap(error[:, 2]); distance = np.linalg.norm(error[:, :2], axis=1)
    covariance = np.column_stack([predictions[f'p{i}{j}'] for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
    if not np.all(np.isfinite(covariance)) or np.any(np.linalg.eigvalsh(covariance) <= 0):
        raise ValueError('Invalid output covariance')
    nees = np.einsum('ni,ni->n', error, np.linalg.solve(covariance, error[..., None])[..., 0])
    idx = np.searchsorted(t, t+1)
    keep = idx < len(t); origin = np.flatnonzero(keep); idx = idx[keep]
    keep = np.abs(t[idx]-t[origin]-1) < .05; origin, idx = origin[keep], idx[keep]
    delta = relative(estimated[origin], estimated[idx])-relative(gt[origin], gt[idx]); delta[:, 2] = wrap(delta[:, 2])
    bad = (distance > .5) | (np.abs(error[:, 2]) > np.deg2rad(15))
    # Qualified loss intervals must last >1 s; final unrecovered interval is censored.
    intervals, recovered = [], []; start = None
    for i, b in enumerate(bad):
        if b and start is None: start = i
        if start is not None and (not b or i == len(bad)-1):
            duration = t[i]-t[start]
            if duration > 1:
                intervals.append(duration)
                if not b: recovered.append(duration)
            start = None
    scans = rows[rows['kind'] == 'S']; matched = scans[np.isfinite(scans['mx'])]
    latency = rows['latency_us']/1000
    return dict(samples=len(t), synchronized_fraction=float(valid.mean()),
                ate_rmse_m=float(np.sqrt(np.mean(distance**2))), position_p95_m=float(np.percentile(distance, 95)),
                rpe_1s_translation_rmse_m=float(np.sqrt(np.mean(np.sum(delta[:, :2]**2, axis=1)))) if len(delta) else None,
                rpe_1s_yaw_rmse_rad=float(np.sqrt(np.mean(wrap(delta[:, 2])**2))) if len(delta) else None,
                nees_mean=float(nees.mean()), nees_outside_95_fraction=float(np.mean((nees < .216) | (nees > 9.348))),
                nis_mean=float(matched['nis'].mean()) if len(matched) else None,
                nis_outside_95_fraction=float(np.mean((matched['nis'] < .216) | (matched['nis'] > 9.348))) if len(matched) else None,
                loss_fraction=float(sum(intervals)/(t[-1]-t[0])), loss_episodes=len(intervals),
                recovery_seconds=recovered, unrecovered_at_end=bool(bad[-1]),
                scan_acceptance_fraction=float(scans['accepted'].mean()) if len(scans) else None,
                fallback_fraction=float(scans['fallback'].mean()) if len(scans) else None,
                kernel_latency_ms_p50_p95_p99=np.percentile(latency, [50, 95, 99]).tolist(),
                kernel_over_20ms_fraction=float(np.mean(latency > 20)))


def trajectory_plot(dataset, rows, output):
    predictions = rows[rows['kind'] == 'W']
    gt, valid = truth_at(dataset[0], predictions['stamp'])
    xy = np.column_stack([predictions['x'], predictions['y']])[valid]
    gt = gt[valid, :2]
    if not len(xy): return
    both = np.concatenate([xy, gt]); low = both.min(0); high = both.max(0)
    scale = min(720/max(high[0]-low[0], .1), 420/max(high[1]-low[1], .1))
    def points(data):
        a = (data-low)*scale
        return ' '.join(f'{40+x:.2f},{460-y:.2f}' for x, y in a[::max(1, len(a)//3000)])
    output.write_text('<svg xmlns="http://www.w3.org/2000/svg" width="800" height="500">'
                     '<rect width="800" height="500" fill="white"/>'
                     '<text x="30" y="22">Map-frame trajectory (no alignment): black GT, blue estimate</text>'
                     f'<polyline points="{points(gt)}" fill="none" stroke="black"/>'
                     f'<polyline points="{points(xy)}" fill="none" stroke="#1674ba"/></svg>')


def evaluate(args):
    items = [load_dataset(p) for p in args.datasets]
    training = json.loads((args.model_dir/'training.json').read_text())
    model = args.model_dir/'model.txt'
    if hashlib.sha256(model.read_bytes()).hexdigest() != training['model_sha256']:
        raise ValueError('Model hash mismatch')
    validate_splits(items+[(None, m) for m in training['datasets']])
    if any(m['split'] != 'test' for _, m in items):
        raise ValueError('Final evaluation requires held-out test datasets')
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for i, item in enumerate(items):
        result = dict(dataset=item[1], methods={})
        for method in ('classical', 'learned'):
            rows = replay(args.replay, item, args.output/f'{i}_{method}.csv', method, model,
                          training['classical_scale'])
            result['methods'][method] = metrics(item, rows)
            trajectory_plot(item, rows, args.output/f'{i}_{method}.svg')
        results.append(result)
    (args.output/'metrics.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    lines = ['# G1 held-out A/B evaluation', '', 'Ground truth used only by evaluator; no pose alignment.',
             'Kernel latency excludes ROS transport/scheduling and is not end-to-end callback latency.',
             'Map IoU, SLAM build time and loop closure require separate map evidence.', '']
    for r in results:
        lines += [f"## {r['dataset']['world']} / {r['dataset']['trajectory']} / {r['dataset']['scenario']}", '',
                  '|Method|ATE RMSE m|p95 m|Loss fraction|Fallback fraction|', '|---|---:|---:|---:|---:|']
        for method, m in r['methods'].items():
            lines.append(f"|{method}|{m['ate_rmse_m']:.4f}|{m['position_p95_m']:.4f}|{m['loss_fraction']:.3f}|{m['fallback_fraction']}|")
        lines.append('')
    (args.output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('train', 'evaluate'):
        p = sub.add_parser(name);p.add_argument('--replay', type=Path, required=True)
        p.add_argument('--output', type=Path, required=True);p.add_argument('datasets', nargs='+', type=Path)
        if name == 'train':
            p.add_argument('--epochs', type=int, default=500);p.add_argument('--seed', type=int, default=7)
            p.add_argument('--no-history', action='store_true')
        else:p.add_argument('--model-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'train':train(args)
    else:evaluate(args)

if __name__ == '__main__':main()
