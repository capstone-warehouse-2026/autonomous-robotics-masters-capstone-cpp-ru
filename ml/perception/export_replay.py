"""Export paired offline grid replay from recorded RGB-D and clean geometric labels."""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import re

import numpy as np

from evaluate import CONDITIONS, apply
from generate_layout import generate, load_splits, top_level_nodes


def poles_for_layout(layout, seed):
    template = Path(__file__).resolve().parents[2] / 'simulation/worlds/warehouse.wbt'
    world = generate(template.read_text(), layout, seed, '../protos')
    _, nodes = top_level_nodes(world)
    poles = []
    for kind, node in nodes:
        if kind == 'Solid' and re.search(r'name "pole \d+"', node):
            x, y, z = map(float, re.search(r'translation ([^\n]+)', node)[1].split())
            radius = float(re.search(r'radius ([^ }]+)', node)[1])
            poles.append((x, y, z, radius))
    return poles


def sees_pole(depth, labels, pose, camera, poles):
    # Use the same stride as GridBuilder and count only clean obstacle samples.
    v, u = np.where((labels[::2, ::2] == 1) & np.isfinite(depth[::2, ::2]))
    z = depth[2 * v, 2 * u]
    points = np.stack(((2 * u - camera['cx']) * z / camera['fx'],
                       (2 * v - camera['cy']) * z / camera['fy'], z), axis=1)
    world = points @ pose[:3, :3].T + pose[:3, 3]
    for x, y, half_height, radius in poles:
        distance = np.hypot(world[:, 0] - x, world[:, 1] - y)
        on_pole = (np.abs(distance - radius) <= 0.015) & (world[:, 2] <= 2 * half_height)
        if np.count_nonzero(on_pole) >= 5:
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--thin-only', action='store_true', help='export clean frames with visible thin poles')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    for condition in ('thin_obstacles',) if args.thin_only else CONDITIONS:
        rng = np.random.default_rng(args.seed)
        path = args.out / f'{condition}.rgbd'
        digest = hashlib.sha256()
        frames = []
        with path.open('wb') as stream:
            stream.write(b'G2RGBD01')
            for layout in load_splits()['test']:
                root = args.data_root / layout
                meta = json.loads((root / 'meta.json').read_text())
                camera = meta['camera']
                sources = sorted(root.glob('frame_*.npz'))
                if not sources or len(sources) != meta['samples']:
                    raise ValueError(f'incomplete recording: {root}')
                poles = poles_for_layout(layout, meta['seed']) if args.thin_only else None
                for source in sources:
                    with np.load(source) as frame, np.load(root / 'labels' / source.name) as label_file:
                        clean = frame['depth']
                        rgb, depth = apply(condition, frame['rgb'], clean, rng)
                        labels = label_file['labels']
                        base = np.asarray(meta['base_from_camera'])
                        pose = frame['world_from_base']
                        if poles is not None and not sees_pole(clean, labels, pose @ base, camera, poles):
                            continue
                        h, w = clean.shape
                        if labels.shape != (h, w) or rgb.shape != (h, w, 3):
                            raise ValueError(f'inconsistent frame: {source}')
                        header = struct.pack('<II4d32d', h, w, *(camera[k] for k in ('fx', 'fy', 'cx', 'cy')),
                                             *base.ravel(), *pose.ravel())
                        for block in (header, rgb.tobytes(), depth.astype('<f4').tobytes(),
                                      clean.astype('<f4').tobytes(), labels.tobytes()):
                            stream.write(block)
                            digest.update(block)
                        frames.append(f'{layout}/{source.stem}')
        if not frames:
            raise ValueError(f'no frames selected for {condition}')
        manifest = {'condition': condition, 'seed': args.seed, 'frames': frames,
                    'payload_sha256': digest.hexdigest()}
        path.with_suffix('.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
