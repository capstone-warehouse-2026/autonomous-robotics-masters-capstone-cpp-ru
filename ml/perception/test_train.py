"""Unit tests of model.py and train.py; they need PyTorch and run in the train image:
docker compose --profile train run --rm train python3 -m unittest discover"""

import importlib.util
import unittest

import numpy as np

HAS_TORCH = importlib.util.find_spec('torch') is not None


@unittest.skipUnless(HAS_TORCH, 'PyTorch is only in the train image')
class TrainTest(unittest.TestCase):

    def test_model_keeps_resolution_and_outputs_three_classes(self):
        import torch
        from model import SegNet
        output = SegNet().eval()(torch.zeros(2, 5, 240, 320))
        self.assertEqual(tuple(output.shape), (2, 3, 240, 320))

    def test_augmentation_turns_removed_depth_into_unknown(self):
        from train import augment
        rgb = np.full((480, 640, 3), 128, np.uint8)
        depth = np.full((480, 640), 2.0, np.float32)
        labels = np.zeros((480, 640), np.uint8)
        for seed in range(20):
            out_rgb, out_depth, out_labels = augment(rgb, depth, labels, np.random.default_rng(seed))
            self.assertEqual(out_rgb.shape, rgb.shape)
            self.assertTrue((out_labels[~np.isfinite(out_depth)] == 255).all())
            self.assertTrue((out_labels[np.isfinite(out_depth)] == 0).all())  # labels change only with depth

    def test_scores_false_free_rate(self):
        import torch
        from train import confusion, scores
        target = torch.tensor([1, 1, 1, 1, 0, 2])
        prediction = torch.tensor([0, 1, 1, 1, 0, 2])  # one of four obstacle pixels predicted free
        result = scores(confusion(prediction, target))
        self.assertAlmostEqual(result['false_free'], 0.25)
        self.assertAlmostEqual(result['iou_free'], 0.5)


if __name__ == '__main__':
    unittest.main()
