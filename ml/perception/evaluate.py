"""Evaluates the exported ONNX model of method B on the test layouts under fixed conditions (offline).

The model runs through ONNX Runtime, the same artifact CnnClassifier loads. Every condition is applied
to every test frame with a fixed seed, so runs are comparable: clean frames, low light (image x0.3),
depth noise (sigma = 0.005 m * z^2) and depth holes (blobs and 3 % scattered pixels). As at runtime,
pixels without depth are unknown in the ground truth. Results: <model>/evaluation.csv and a table.
This is a development check of the learned method; the final A/B comparison is computed in C++.
"""

import argparse
import csv
import glob
import json
import os

import numpy as np
import onnxruntime

from generate_layout import load_splits
from preprocess import CLASSES, network_input, network_target
from train import CLASS_NAMES

CONDITIONS = ('clean', 'low_light', 'depth_noise', 'depth_holes')


def apply(condition, rgb, depth, rng):
    rgb, depth = rgb.copy(), depth.copy()
    if condition == 'low_light':
        rgb = (rgb.astype(np.float32) * 0.3).astype(np.uint8)
    elif condition == 'depth_noise':
        depth = depth + rng.normal(0.0, 1.0, depth.shape).astype(np.float32) * 0.005 * depth ** 2
    elif condition == 'depth_holes':
        height, width = depth.shape
        for _ in range(4):
            h, w = rng.integers(20, 80, 2)
            y, x = rng.integers(0, height - h), rng.integers(0, width - w)
            depth[y:y + h, x:x + w] = np.nan
        depth[rng.random(depth.shape) < 0.03] = np.nan
    return rgb, depth


def metrics(matrix):
    matrix = matrix.astype(np.float64)
    tp = np.diag(matrix)
    iou = tp / np.maximum(matrix.sum(0) + matrix.sum(1) - tp, 1)
    obstacle_precision = matrix[1, 1] / max(matrix[:, 1].sum(), 1)
    obstacle_recall = matrix[1, 1] / max(matrix[1].sum(), 1)
    return {**{f'iou_{name}': iou[i] for i, name in enumerate(CLASS_NAMES)},
            'miou': iou.mean(), 'false_free': matrix[1, 0] / max(matrix[1].sum(), 1),
            'obstacle_precision': obstacle_precision, 'obstacle_recall': obstacle_recall}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default='../../data/g2/models/latest')
    parser.add_argument('--data-root', default='../../data/g2')
    parser.add_argument('--layouts', nargs='*', help='default: the test split of splits.json')
    parser.add_argument('--seed', type=int, default=0, help='seed of the conditions')
    args = parser.parse_args()

    layouts = args.layouts or list(load_splits()['test'])
    session = onnxruntime.InferenceSession(os.path.join(args.model, 'model.onnx'), providers=['CPUExecutionProvider'])
    rows = []
    for condition in CONDITIONS:
        rng = np.random.default_rng(args.seed)
        matrix = np.zeros((CLASSES, CLASSES), np.int64)
        frames = 0
        for layout in layouts:
            for path in sorted(glob.glob(os.path.join(args.data_root, layout, 'frame_*.npz'))):
                frame = np.load(path)
                labels = np.load(os.path.join(os.path.dirname(path), 'labels', os.path.basename(path)))['labels']
                rgb, depth = apply(condition, frame['rgb'], frame['depth'], rng)
                labels = labels.copy()
                labels[~np.isfinite(depth)] = 255
                logits = session.run(['logits'], {'input': network_input(rgb, depth)[None]})[0][0]
                target = network_target(labels)
                matrix += np.bincount(target.ravel() * CLASSES + logits.argmax(0).ravel(),
                                      minlength=CLASSES * CLASSES).reshape(CLASSES, CLASSES)
                frames += 1
        rows.append({'condition': condition, 'frames': frames, **metrics(matrix)})
        print(f'{condition:12s} {frames} frames  obstacle IoU {rows[-1]["iou_obstacle"]:.3f}  '
              f'free IoU {rows[-1]["iou_free"]:.3f}  false-free {rows[-1]["false_free"]:.4f}  '
              f'obstacle recall {rows[-1]["obstacle_recall"]:.3f}', flush=True)

    with open(os.path.join(args.model, 'evaluation.csv'), 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(args.model, 'evaluation.json'), 'w') as stream:
        json.dump({'layouts': layouts, 'seed': args.seed, 'results': rows}, stream, indent=2)


if __name__ == '__main__':
    main()
