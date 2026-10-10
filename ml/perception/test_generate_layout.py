"""Unit tests of generate_layout.py: python3 -m unittest discover ml/perception"""

import os
import unittest

from generate_layout import HERE, build_layout, generate, load_splits, top_level_nodes

TEMPLATE = os.path.join(HERE, '..', '..', 'simulation', 'worlds', 'warehouse.wbt')


def read_template():
    with open(TEMPLATE) as stream:
        return stream.read()


class GenerateLayoutTest(unittest.TestCase):

    def test_same_seed_same_world_and_different_seeds_differ(self):
        template = read_template()
        self.assertEqual(generate(template, 'a', 7, '../protos'), generate(template, 'a', 7, '../protos'))
        self.assertNotEqual(generate(template, 'a', 7, '../protos'), generate(template, 'a', 8, '../protos'))

    def test_splits_do_not_share_layouts_or_seeds(self):
        splits = load_splits()
        ids = [layout for layouts in splits.values() for layout in layouts]
        seeds = [seed for layouts in splits.values() for seed in layouts.values()]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(len(seeds), len(set(seeds)))
        self.assertGreaterEqual(len(splits['train']), 3)
        self.assertGreaterEqual(len(splits['val']), 1)
        self.assertGreaterEqual(len(splits['test']), 2)

    def test_objects_stay_inside_and_do_not_overlap(self):
        for seed in (0, 1, 2, 1000, 2000, 2001):
            layout = build_layout(seed, 9.85, 9.85)
            for index, (x0, y0, x1, y1) in enumerate(layout.rects):
                self.assertTrue(-9.85 <= x0 < x1 <= 9.85 and -9.85 <= y0 < y1 <= 9.85)
                for a0, b0, a1, b1 in layout.rects[index + 1:]:
                    self.assertTrue(x1 <= a0 or a1 <= x0 or y1 <= b0 or b1 <= y0, f'overlap, seed {seed}')

    def test_world_keeps_robot_and_replaces_contents(self):
        world = generate(read_template(), 'train_0', 0, '/opt/protos')
        _, nodes = top_level_nodes(world)
        kinds = [kind for kind, _ in nodes]
        self.assertEqual(kinds.count('TiagoBase'), 1)
        self.assertEqual(kinds.count('RectangleArena'), 1)
        self.assertGreater(kinds.count('WarehouseRack'), 0)
        self.assertIn('Astra {', world)  # the camera comes with the robot of the template
        self.assertIn('EXTERNPROTO "/opt/protos/WarehouseRack.proto"', world)
        self.assertNotIn('"rack 1"\n}\nWarehouseRack {\n  translation -4 5 0', world)  # template racks dropped


if __name__ == '__main__':
    unittest.main()
