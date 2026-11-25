"""Unit tests of the G3 data pipeline (numpy only; model tests run where PyTorch is installed):
python3 -m unittest discover -s ml/planning"""

import importlib.util
import unittest

import numpy as np

import sim

HAS_TORCH = importlib.util.find_spec('torch') is not None


class SimTest(unittest.TestCase):

    def test_pool_matches_cpp_convention(self):
        # The same example as LearnedInput.PoolFractionWithPartialBlocks in test_learned_cost.cpp.
        fine = np.array([[1, 1, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 0, 0]], np.float32)
        np.testing.assert_allclose(sim.pool(fine), [[2 / 12, 2 / 3]], rtol=1e-6)

    def test_episode_is_deterministic_and_labels_are_valid(self):
        a = sim.episode(5, 3, probes=20, agents_range=(3, 4))
        b = sim.episode(5, 3, probes=20, agents_range=(3, 4))
        np.testing.assert_array_equal(a['input'], b['input'])
        np.testing.assert_array_equal(np.nan_to_num(a['label'], nan=-1), np.nan_to_num(b['label'], nan=-1))
        self.assertEqual(a['input'].shape, (len(sim.CHANNELS), 100, 100))
        self.assertTrue(((a['input'] >= 0) & (a['input'] <= 1)).all())
        label = a['label'][np.isfinite(a['label'])]
        self.assertGreater(label.size, 0)
        self.assertTrue(((label >= 0) & (label <= sim.MAX_LABEL)).all())

    def test_observed_occupancy_only_inside_window_and_off_static(self):
        e = sim.episode(1, 0, probes=10)
        observed, area, static = e['input'][1:1 + sim.HISTORY], e['input'][-1], e['input'][0]
        self.assertTrue((observed[:, area == 0] == 0).all())
        self.assertTrue((observed <= 1.0 - static + 1e-6).all())

    def test_controller_waits_for_agent_ahead(self):
        path = np.stack([np.linspace(-5, 5, 201), np.zeros(201)], axis=1)
        steps = 300
        agents = np.zeros((steps, 1, 2))
        agents[:150, 0] = (0.0, 0.0)      # standing on the path for 15 s
        agents[150:, 0] = (0.0, 5.0)      # then leaves
        logs, done = sim.run_probes(None, [path], agents, 0)
        waited = sum(w for _, _, w in logs[0]) * sim.DT
        self.assertTrue(done[0])
        # Reaches the stopping distance after (5 - 1) m / 0.5 m/s = 8 s and waits until 15 s.
        self.assertAlmostEqual(waited, 7.0, delta=0.3)


@unittest.skipUnless(HAS_TORCH, 'PyTorch is only in the train image')
class ModelTest(unittest.TestCase):

    def test_output_is_non_negative_and_keeps_size(self):
        import torch
        from model import CostNet
        y = CostNet()(torch.randn(2, len(sim.CHANNELS), 37, 51))
        self.assertEqual(tuple(y.shape), (2, 1, 37, 51))
        self.assertTrue((y >= 0).all())


if __name__ == '__main__':
    unittest.main()
