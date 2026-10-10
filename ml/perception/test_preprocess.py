"""Unit tests of preprocess.py (numpy only): python3 -m unittest discover ml/perception"""

import unittest

import numpy as np

from preprocess import MAX_DEPTH, UNKNOWN_CLASS, labels_from_logits, network_input, network_target


class PreprocessTest(unittest.TestCase):

    def test_shapes_and_channels(self):
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[0:2, 0:2] = [[[0, 100, 255], [100, 100, 255]], [[0, 100, 255], [100, 100, 255]]]
        depth = np.full((480, 640), 2.0, np.float32)
        tensor = network_input(rgb, depth)
        self.assertEqual(tensor.shape, (5, 240, 320))
        self.assertEqual(tensor.dtype, np.float32)
        np.testing.assert_allclose(tensor[0:3, 0, 0], [50 / 255, 100 / 255, 1.0], rtol=1e-6)  # 2x2 mean
        self.assertAlmostEqual(float(tensor[3, 0, 0]), 2.0 / MAX_DEPTH)
        self.assertEqual(float(tensor[4, 0, 0]), 1.0)

    def test_depth_uses_top_left_pixel_and_marks_invalid(self):
        rgb = np.zeros((480, 640, 3), np.uint8)
        depth = np.full((480, 640), 1.0, np.float32)
        depth[0, 0] = np.nan       # top-left of block (0, 0)
        depth[0, 3] = 4.0          # not the top-left of block (0, 1): ignored
        depth[2, 2] = 20.0         # beyond 8 m: clipped
        tensor = network_input(rgb, depth)
        self.assertEqual(float(tensor[3, 0, 0]), 0.0)
        self.assertEqual(float(tensor[4, 0, 0]), 0.0)
        self.assertAlmostEqual(float(tensor[3, 0, 1]), 1.0 / MAX_DEPTH)
        self.assertEqual(float(tensor[3, 1, 1]), 1.0)

    def test_target_maps_unknown_and_samples_like_depth(self):
        labels = np.zeros((480, 640), np.uint8)
        labels[0, 0] = 255
        labels[0, 3] = 1  # not sampled
        labels[2, 2] = 1
        target = network_target(labels)
        self.assertEqual(target.shape, (240, 320))
        self.assertEqual(target[0, 0], UNKNOWN_CLASS)
        self.assertEqual(target[0, 1], 0)
        self.assertEqual(target[1, 1], 1)

    def test_labels_from_logits_upsamples_and_keeps_nan_unknown(self):
        logits = np.zeros((3, 240, 320), np.float32)
        logits[1, 0, 0] = 1.0      # obstacle
        logits[2, 0, 1] = 1.0      # unknown
        depth = np.full((480, 640), 1.0, np.float32)
        depth[1, 1] = np.nan       # inside block (0, 0)
        labels = labels_from_logits(logits, depth)
        self.assertEqual(labels.shape, (480, 640))
        self.assertEqual(labels.dtype, np.uint8)
        self.assertEqual(labels[0:2, 0:2].tolist(), [[1, 1], [1, 255]])
        self.assertTrue((labels[0:2, 2:4] == 255).all())
        self.assertTrue((labels[0:2, 4:6] == 0).all())  # ties go to the first class, free

    def test_rejects_other_sizes(self):
        with self.assertRaises(ValueError):
            network_input(np.zeros((240, 320, 3), np.uint8), np.zeros((240, 320), np.float32))


if __name__ == '__main__':
    unittest.main()
