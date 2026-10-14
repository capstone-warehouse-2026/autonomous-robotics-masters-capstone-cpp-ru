"""Writes ground-truth pixel labels of a recorded dataset to <dataset>/labels/ (offline tool).

Labels come from the clean recorded depth and the exact camera pose (world_from_base of the frame
times base_from_camera of meta.json); see labels.compute_labels. Thresholds can be changed and the
labels regenerated without recording again.
"""

import argparse
import glob
import json
import os

import numpy as np

from labels import FREE, OBSTACLE, UNKNOWN, compute_labels


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('dataset', help='directory written by record_dataset.py')
    parser.add_argument('--floor-tolerance', type=float, default=0.02, help='m, |height| treated as floor')
    parser.add_argument('--robot-height', type=float, default=1.0, help='m, higher points are passable')
    args = parser.parse_args()

    with open(os.path.join(args.dataset, 'meta.json')) as stream:
        meta = json.load(stream)
    camera = meta['camera']
    base_from_camera = np.array(meta['base_from_camera'])
    out = os.path.join(args.dataset, 'labels')
    os.makedirs(out, exist_ok=True)

    counts = {FREE: 0, OBSTACLE: 0, UNKNOWN: 0}
    frames = sorted(glob.glob(os.path.join(args.dataset, 'frame_*.npz')))
    for path in frames:
        frame = np.load(path)
        world_from_camera = frame['world_from_base'] @ base_from_camera
        labels = compute_labels(frame['depth'], camera, world_from_camera,
                                args.floor_tolerance, args.robot_height)
        np.savez_compressed(os.path.join(out, os.path.basename(path)), labels=labels)
        for value in counts:
            counts[value] += int((labels == value).sum())

    total = max(1, sum(counts.values()))
    summary = {
        'frames': len(frames),
        'floor_tolerance': args.floor_tolerance,
        'robot_height': args.robot_height,
        'pixel_fraction': {'free': counts[FREE] / total, 'obstacle': counts[OBSTACLE] / total,
                           'unknown': counts[UNKNOWN] / total},
    }
    with open(os.path.join(out, 'meta.json'), 'w') as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
