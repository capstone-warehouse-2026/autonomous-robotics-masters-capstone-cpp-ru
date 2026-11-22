"""Package G2 offline grid results and compute paired frame-bootstrap intervals."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'data/g2'
CASES = ('clean', 'low_light', 'depth_noise', 'depth_holes', 'thin_obstacles')
CLASSES = ('free', 'obstacle', 'unknown')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def counts(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if [int(row['frame']) for row in rows] != list(range(len(rows))):
        raise ValueError(f'non-contiguous frame IDs in {path}')
    return np.array([[int(row[f'gt_{gt}_pred_{pred}']) for gt in CLASSES for pred in CLASSES]
                     for row in rows], dtype=np.int64)


def scores(values):
    tp = values[..., 4]
    iou = tp / (tp + values[..., 1] + values[..., 3] + values[..., 5])
    false_free = values[..., 3] / (tp + values[..., 3] + values[..., 5])
    return np.stack((iou, false_free), axis=-1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--image-id', required=True)
    parser.add_argument('--repeats', type=int, default=5000)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    manifest = {
        'protocol': 'offline replay, conditional on recorded test layouts',
        'packaged_from_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'packaged_from_dirty_tree': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT)),
        'distance_run_image_id': args.image_id,
        'packaging_host': platform.platform(),
        'bootstrap': {'seed': 12345, 'repeats': args.repeats, 'unit': 'paired frame, stratified by layout'},
        'models': {}, 'inputs': {}, 'results': {}, 'source_sha256': {},
    }
    for relative in ('ml/perception/export_replay.py', 'ml/perception/summarize_grid_report.py',
                     'ros2_ws/src/capstone_perception/src/grid_replay.cpp',
                     'ros2_ws/src/capstone_perception/config/perception.yaml'):
        manifest['source_sha256'][relative] = sha256(ROOT / relative)
    models = {
        'learned': ROOT / 'ros2_ws/src/capstone_perception/models/perception_cnn.onnx',
        'without_rgb': DATA / 'models/without_rgb/model.onnx',
    }
    for method, path in models.items():
        manifest['models'][method] = {'sha256': sha256(path), 'rgb_zeroed': method == 'without_rgb'}
    rng = np.random.default_rng(12345)
    with (args.out / 'paired_intervals.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 'metric', 'rgbd', 'without_rgb', 'delta', 'lo95', 'hi95'])
        for case in CASES:
            replay = DATA / ('grid_replay_thin' if case == 'thin_obstacles' else 'grid_replay')
            metadata = json.loads((replay / f'{case}.json').read_text())
            paths = [replay / f'{case}.{method}.csv' for method in models]
            a, b = map(counts, paths)
            groups = np.array([frame.split('/')[0] for frame in metadata['frames']])
            if len(groups) != len(a) or a.shape != b.shape:
                raise ValueError(f'frame mismatch in {case}')
            manifest['inputs'][case] = metadata
            manifest['results'][case] = {path.name: sha256(path) for path in paths}
            draws = np.zeros((2, args.repeats, 9), dtype=np.int64)
            for group in sorted(set(groups)):
                indices = np.flatnonzero(groups == group)
                sampled = rng.choice(indices, size=(args.repeats, len(indices)))
                draws[0] += a[sampled].sum(axis=1)
                draws[1] += b[sampled].sum(axis=1)
            observed_a, observed_b = scores(a.sum(axis=0)), scores(b.sum(axis=0))
            bounds = np.quantile(scores(draws[0]) - scores(draws[1]), [0.025, 0.975], axis=0)
            for i, metric in enumerate(('obstacle_iou', 'false_free')):
                writer.writerow([case, metric, observed_a[i], observed_b[i], observed_a[i] - observed_b[i],
                                 bounds[0, i], bounds[1, i]])
            for method, matrix in zip(models, (a, b)):
                with (args.out / f'{case}.{method}.confusion.csv').open('w') as out:
                    csv_out = csv.writer(out)
                    csv_out.writerow(['gt/pred', *CLASSES])
                    for label, row in zip(CLASSES, matrix.sum(axis=0).reshape(3, 3)):
                        csv_out.writerow([label, *row])

    figure, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    for directory, label in [('latest', 'RGB-D'), ('without_rgb', 'Без RGB')]:
        with (DATA / 'models' / directory / 'metrics.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        epochs = [int(row['epoch']) for row in rows]
        axes[0].plot(epochs, [float(row['val_iou_obstacle']) for row in rows], label=label)
        axes[1].plot(epochs, [100 * float(row['val_false_free']) for row in rows], label=label)
    for ax, ylabel in zip(axes, ('Пиксельный IoU препятствий', 'Ложно-свободные пиксели, %')):
        ax.set(xlabel='Эпоха', ylabel=ylabel)
        ax.grid(alpha=0.3)
        ax.legend()
    figure.savefig(args.out / 'training.svg')
    plt.close(figure)
    for case in CASES[:-1]:
        distance = DATA / 'grid_replay' / f'{case}.report.learned.csv.distance.csv'
        (args.out / distance.name).write_bytes(distance.read_bytes())
        manifest['results'][case][distance.name] = sha256(distance)
    (args.out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
