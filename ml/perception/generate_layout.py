"""Procedural warehouse layouts for the training data of method B (offline data preparation).

The robot, its camera, floor, walls and lights are taken unchanged from a template world
(simulation/worlds/warehouse.wbt), so the robot configuration stays defined in one place; only the
contents of the warehouse are generated from the layout seed: rows of racks of random size, goods on
the shelves, boxes, pallets and crates on the floor, thin poles and low obstacles (hard cases of G2),
yellow station markers and the floor and wall tint. Only PROTOs stored in the repository are used,
so generated worlds load without network.

Layout IDs and seeds of the train/val/test splits are fixed in splits.json (docs/EXPERIMENTS.md).
Uses the standard library only, so `--list` runs on the host as well.
"""

import argparse
import json
import math
import os
import random
import re

HERE = os.path.dirname(os.path.abspath(__file__))
KEPT_NODES = ('WorldInfo', 'Viewpoint', 'TexturedBackground', 'TexturedBackgroundLight', 'RectangleArena',
              'TiagoBase')
CORRIDOR = 2.0  # m kept free between the racks and the walls


def load_splits(path=os.path.join(HERE, 'splits.json')):
    with open(path) as stream:
        return json.load(stream)


def layout_seed(layout_id, splits):
    for layouts in splits.values():
        if layout_id in layouts:
            return layouts[layout_id]
    raise SystemExit(f'unknown layout {layout_id!r}; see splits.json')


def top_level_nodes(world_text):
    """Splits a .wbt file into its header lines and top-level node blocks [(type, text)]."""
    header, nodes, block = [], [], None
    for line in world_text.splitlines():
        if block is None:
            match = re.match(r'^(?:DEF \S+ )?(\w+) \{$', line)
            if match:
                block = (match.group(1), [line])
            elif line.startswith('#VRML') or line.startswith('EXTERNPROTO'):
                header.append(line)
            continue
        block[1].append(line)
        if line == '}':
            nodes.append((block[0], '\n'.join(block[1])))
            block = None
    return header, nodes


class Layout:
    """Objects of one generated warehouse; rects are axis-aligned footprints (x0, y0, x1, y1)."""

    def __init__(self, seed, half_x, half_y):
        self.seed = seed
        self.rng = random.Random(seed)
        self.half_x = half_x
        self.half_y = half_y
        self.rects = []
        self.nodes = []
        self.names = {}

    def name(self, kind):
        self.names[kind] = self.names.get(kind, 0) + 1
        return f'{kind} {self.names[kind]}'

    def fits(self, rect, margin):
        x0, y0, x1, y1 = rect
        if x0 < -self.half_x or y0 < -self.half_y or x1 > self.half_x or y1 > self.half_y:
            return False
        return all(x1 + margin <= a0 or a1 + margin <= x0 or y1 + margin <= b0 or b1 + margin <= y0
                   for a0, b0, a1, b1 in self.rects)

    def place(self, width, depth, margin, attempts=200):
        """Random free centre for a width x depth footprint; None when the floor is too crowded."""
        for _ in range(attempts):
            x = self.rng.uniform(-self.half_x + width / 2, self.half_x - width / 2)
            y = self.rng.uniform(-self.half_y + depth / 2, self.half_y - depth / 2)
            rect = (x - width / 2, y - depth / 2, x + width / 2, y + depth / 2)
            if self.fits(rect, margin):
                self.rects.append(rect)
                return x, y
        return None


def fmt(*values):
    return ' '.join(f'{value:.4g}' for value in values)


def colour(rng, low=0.15, high=0.95):
    return fmt(*(rng.uniform(low, high) for _ in range(3)))


def add_racks(layout):
    rng = layout.rng
    along_x = rng.random() < 0.5
    half_along, half_across = (layout.half_x, layout.half_y) if along_x else (layout.half_y, layout.half_x)
    depth = rng.uniform(0.6, 1.0)
    height = rng.uniform(1.6, 2.4)
    shelves = rng.randint(2, 4)
    aisle = rng.uniform(1.8, 3.0)
    span = 2 * (half_across - CORRIDOR)
    rows = max(2, min(6, int((span + aisle) // (depth + aisle))))
    first = -((rows - 1) * (depth + aisle)) / 2
    for row in range(rows):
        across = first + row * (depth + aisle)
        position = -half_along + CORRIDOR + rng.uniform(0.0, 1.0)
        while True:
            length = rng.uniform(2.0, 4.0)
            if position + length > half_along - CORRIDOR:
                break
            centre = position + length / 2
            x, y = (centre, across) if along_x else (across, centre)
            size_x, size_y = (length, depth) if along_x else (depth, length)
            layout.rects.append((x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2))
            yaw = 0.0 if along_x else math.pi / 2
            layout.nodes.append(
                f'WarehouseRack {{\n  translation {fmt(x, y, 0)}\n  rotation 0 0 1 {yaw:.6g}\n'
                f'  name "{layout.name("rack")}"\n  size {fmt(length, depth, height)}\n  shelves {shelves}\n}}')
            add_shelf_goods(layout, x, y, along_x, length, depth, height, shelves)
            position += length + rng.uniform(1.5, 2.5)


def add_shelf_goods(layout, x, y, along_x, length, depth, height, shelves):
    """Boxes on the shelves; shelf geometry as in simulation/protos/WarehouseRack.proto."""
    rng = layout.rng
    step = (height - 0.25) / (shelves - 1)
    for _ in range(rng.randint(0, 4)):
        shelf = rng.randrange(shelves)
        top = 0.15 + shelf * step + 0.02
        size = rng.uniform(0.25, min(0.5, depth * 0.8))
        if shelf < shelves - 1 and top + size > 0.15 + (shelf + 1) * step - 0.02:
            continue  # does not fit under the next shelf
        offset = rng.uniform(-(length / 2 - size / 2 - 0.1), length / 2 - size / 2 - 0.1)
        bx, by = (x + offset, y) if along_x else (x, y + offset)
        proto = rng.choice(['CardboardBox', 'WoodenBox'])
        layout.nodes.append(f'{proto} {{\n  translation {fmt(bx, by, top + size / 2)}\n'
                            f'  name "{layout.name("shelf box")}"\n  size {fmt(size, size, size)}\n}}')


def add_floor_objects(layout):
    rng = layout.rng
    for _ in range(rng.randint(6, 14)):
        kind = rng.choice(['cardboard', 'wooden', 'crates', 'pallet', 'pallet stack'])
        turned = rng.random() < 0.5
        yaw = math.pi / 2 if turned else 0.0
        if kind == 'cardboard':
            w, d, h = (rng.uniform(0.3, 0.8) for _ in range(3))
        elif kind == 'wooden':
            w = d = h = rng.uniform(0.4, 0.8)
        elif kind == 'crates':
            w, d, h = 0.6, 0.4, 0.3
        else:
            w, d, h = 1.2, 0.8, 0.14
        footprint = (d, w) if turned else (w, d)
        spot = layout.place(*footprint, margin=0.3)
        if spot is None:
            continue
        x, y = spot
        name = layout.name(kind)
        if kind == 'cardboard':
            layout.nodes.append(f'CardboardBox {{\n  translation {fmt(x, y, h / 2)}\n  rotation 0 0 1 {yaw:.6g}\n'
                                f'  name "{name}"\n  size {fmt(w, d, h)}\n}}')
        elif kind == 'wooden':
            layout.nodes.append(f'WoodenBox {{\n  translation {fmt(x, y, h / 2)}\n  rotation 0 0 1 {yaw:.6g}\n'
                                f'  name "{name}"\n  size {fmt(w, d, h)}\n}}')
        elif kind == 'crates':
            for level in range(rng.randint(1, 3)):
                layout.nodes.append(f'PlasticCrate {{\n  translation {fmt(x, y, 0.3 * level)}\n'
                                    f'  rotation 0 0 1 {yaw:.6g}\n  name "{name} {level}"\n'
                                    f'  size 0.6 0.4 0.3\n  color {colour(rng, 0.1, 0.9)}\n}}')
        elif kind == 'pallet':
            layout.nodes.append(f'WoodenPallet {{\n  translation {fmt(x, y, 0)}\n  rotation 0 0 1 {yaw:.6g}\n'
                                f'  name "{name}"\n}}')
        else:
            layout.nodes.append(f'WoodenPalletStack {{\n  translation {fmt(x, y, 0)}\n  rotation 0 0 1 {yaw:.6g}\n'
                                f'  name "{name}"\n  palletNumber {rng.randint(2, 8)}\n}}')


def solid(name, x, y, z, geometry, rgb):
    return (f'Solid {{\n  translation {fmt(x, y, z)}\n  children [\n    Shape {{\n      appearance PBRAppearance {{\n'
            f'        baseColor {rgb}\n        roughness 0.6\n        metalness 0.2\n      }}\n'
            f'      geometry {geometry}\n    }}\n  ]\n  name "{name}"\n  boundingObject {geometry}\n}}')


def add_hard_cases(layout):
    """Thin poles (2-5 cm) and low obstacles (4-15 cm): the hardest cases for obstacle detection."""
    rng = layout.rng
    for _ in range(rng.randint(2, 6)):
        radius, height = rng.uniform(0.02, 0.05), rng.uniform(0.5, 1.8)
        spot = layout.place(2 * radius, 2 * radius, margin=0.6)
        if spot:
            layout.nodes.append(solid(layout.name('pole'), *spot, height / 2,
                                      f'Cylinder {{ height {height:.4g} radius {radius:.4g} }}', colour(rng)))
    for _ in range(rng.randint(1, 4)):
        w, d, h = rng.uniform(0.2, 0.6), rng.uniform(0.2, 0.6), rng.uniform(0.04, 0.15)
        spot = layout.place(w, d, margin=0.6)
        if spot:
            layout.nodes.append(solid(layout.name('low obstacle'), *spot, h / 2,
                                      f'Box {{ size {fmt(w, d, h)} }}', colour(rng)))


def add_station_markers(layout):
    rng = layout.rng
    for _ in range(rng.randint(2, 6)):
        spot = layout.place(1.2, 1.2, margin=0.2)
        if spot:
            x, y = spot
            layout.nodes.append(
                f'Solid {{\n  translation {fmt(x, y, 0.002)}\n  children [\n    Shape {{\n'
                f'      appearance PBRAppearance {{\n        baseColor 1 0.8 0\n        roughness 1\n'
                f'        metalness 0\n      }}\n      geometry Box {{ size 1.2 1.2 0.004 }}\n    }}\n  ]\n'
                f'  name "{layout.name("station")}"\n}}')


def tint_arena(arena, rng):
    floor = fmt(*([rng.uniform(0.55, 1.0)] * 3))
    arena = arena.replace('floorAppearance RoughConcrete {', f'floorAppearance RoughConcrete {{\n    colorOverride {floor}', 1)
    return re.sub(r'(wallAppearance Roughcast \{\s*colorOverride) [^\n]+', rf'\1 {colour(rng, 0.6, 0.95)}', arena, count=1)


def build_layout(seed, half_x, half_y):
    layout = Layout(seed, half_x, half_y)
    add_racks(layout)
    add_floor_objects(layout)
    add_hard_cases(layout)
    add_station_markers(layout)
    return layout


def generate(template_text, layout_id, seed, protos_dir):
    """World text of one layout: template robot/arena/lights + generated warehouse contents."""
    header, nodes = top_level_nodes(template_text)
    kept = {kind: text for kind, text in nodes if kind in KEPT_NODES}
    missing = [kind for kind in KEPT_NODES if kind not in kept]
    if missing:
        raise SystemExit(f'template world has no {", ".join(missing)}')
    floor = re.search(r'floorSize ([\d.]+) ([\d.]+)', kept['RectangleArena'])
    wall = re.search(r'wallThickness ([\d.]+)', kept['RectangleArena'])
    half_x = float(floor.group(1)) / 2 - float(wall.group(1)) / 2 - 0.05
    half_y = float(floor.group(2)) / 2 - float(wall.group(1)) / 2 - 0.05
    layout = build_layout(seed, half_x, half_y)
    kept['RectangleArena'] = tint_arena(kept['RectangleArena'], layout.rng)
    header = [line.replace('"../protos/', f'"{protos_dir.rstrip("/")}/') for line in header]
    parts = [header[0], f'# Generated by ml/perception/generate_layout.py: layout {layout_id}, seed {seed}.', '']
    parts += header[1:] + ['']
    parts += [kept[kind] for kind in KEPT_NODES] + layout.nodes
    return '\n'.join(parts) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--list', action='store_true', help='print the layout IDs of all splits and exit')
    parser.add_argument('--layout', help='layout ID from splits.json, e.g. train_0')
    parser.add_argument('--template', default=os.path.join(HERE, '..', '..', 'simulation', 'worlds', 'warehouse.wbt'))
    parser.add_argument('--protos-dir', default='../protos', help='path of simulation/protos as seen from the world')
    parser.add_argument('--out', help='world file to write')
    args = parser.parse_args()
    splits = load_splits()
    if args.list:
        print(' '.join(layout for layouts in splits.values() for layout in layouts))
        return
    if not args.layout or not args.out:
        parser.error('--layout and --out are required')
    seed = layout_seed(args.layout, splits)
    with open(args.template) as stream:
        world = generate(stream.read(), args.layout, seed, args.protos_dir)
    with open(args.out, 'w') as stream:
        stream.write(world)
    print(json.dumps({'layout_id': args.layout, 'seed': seed, 'world': args.out}))


if __name__ == '__main__':
    main()
