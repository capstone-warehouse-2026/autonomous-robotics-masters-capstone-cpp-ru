"""Контрактные проверки среды и обучения; не подменяют C++ benchmark."""

import copy
from pathlib import Path
import tempfile
import unittest

from dqn import Network, ReplayBuffer
from env import DispatchEnvironment, make_scenario
import numpy as np
from spec import masked_argmax, observation


class EnvironmentTests(unittest.TestCase):
    def test_future_arrivals_are_not_observed(self):
        scenario = make_scenario('test', 2000, 'high')
        for job in scenario['jobs']:
            job['release_time'] = 10.0
        env = DispatchEnvironment(scenario)
        inputs, mask = env.observe()
        self.assertFalse(inputs[:28].any())
        self.assertEqual(mask.tolist(), [False, False, False, False, True])
        for _ in range(10):
            env.step(4)
        self.assertEqual(len(env.candidates()), 4)
        self.assertEqual(len([job for job in env.jobs if job['state'] == 'released']), 12)

    def test_oldest_four_and_edd(self):
        scenario = make_scenario('test', 2000, 'high')
        for index, job in enumerate(scenario['jobs']):
            job['release_time'] = 0.0
            job['due_time'] = 100.0 - index
        env = DispatchEnvironment(scenario)
        self.assertEqual([job['id'] for job in env.candidates()],
                         ['job_000', 'job_001', 'job_002', 'job_003'])
        self.assertEqual(env.classical_action(), 3)

    def test_wait_is_exactly_one_sim_second(self):
        env = DispatchEnvironment(make_scenario('test', 2000, 'low'))
        env.step(4)
        self.assertEqual(env.now, 1.0)

    def test_masked_action_is_rejected(self):
        env = DispatchEnvironment(make_scenario('test', 2000, 'low'))
        with self.assertRaises(ValueError):
            env.step(3)

    def test_blocked_pickup_has_bounded_attempts(self):
        scenario = make_scenario('test', 2000, 'low')
        first = scenario['jobs'][0]
        first['pickup'] = 's1'
        scenario['blocked'][0][1] = True
        env = DispatchEnvironment(scenario)
        env.step(0)
        self.assertEqual(env.jobs[0]['state'], 'failed')
        self.assertEqual(env.jobs[0]['failures'], 2)
        self.assertEqual(env.navigation_failures, 2)
        self.assertEqual(env.position, 0)
        self.assertEqual(env.now, 2.0)

    def test_previous_failure_is_observed_by_next_job(self):
        env = DispatchEnvironment(make_scenario('test', 2000, 'high'))
        env.navigation_failures = 3
        inputs, _ = env.observe()
        self.assertEqual(inputs[5], 1.5)

    def test_loaded_failure_does_not_start_another_job(self):
        scenario = make_scenario('test', 2000, 'low')
        scenario['jobs'][0].update(pickup='s0', dropoff='s1')
        scenario['blocked'][0][1] = True
        env = DispatchEnvironment(scenario)
        env.step(0)
        self.assertTrue(env.done)
        self.assertEqual(env.failed, 1)

    def test_splits_reject_seed_leakage(self):
        with self.assertRaises(ValueError):
            make_scenario('train', 2000, 'low')
        train = make_scenario('train', 0, 'low')
        test = make_scenario('test', 2000, 'low')
        self.assertNotEqual(train['travel_times'], test['travel_times'])

    def test_observation_contains_service_slack_and_congestion(self):
        job = {'pickup_time': 12.0, 'delivery_time': 24.0, 'service_time': 4.0,
               'due_time': 100.0, 'release_time': 0.0, 'congestion': 2.0, 'failures': 1}
        inputs, _ = observation([job], 10.0, 2.0, -2.0)
        np.testing.assert_allclose(inputs[:7], [0.1, 0.2, 50 / 300, 10 / 300, 2 / 3, 0.5, 1])
        ablated, _ = observation([job], 10.0, 2.0, -2.0, congestion=False)
        self.assertEqual(ablated[4], 0.0)


class LearningTests(unittest.TestCase):
    def test_argmax_never_selects_empty_slot(self):
        self.assertEqual(masked_argmax([1, 100, 3, 4, 0], [True, False, False, False, True]), 0)
        with self.assertRaises(ValueError):
            masked_argmax([1, np.nan, 3, 4, 0], [True, False, False, False, True])

    def test_training_changes_weights_and_reduces_loss(self):
        network = Network(np.random.default_rng(0))
        inputs = np.ones((32, 32), dtype=np.float32)
        actions = np.zeros(32, dtype=int)
        targets = np.full(32, 2.0, dtype=np.float32)
        before = copy.deepcopy(network.parameters)
        losses = [network.update(inputs, actions, targets) for _ in range(100)]
        self.assertLess(losses[-1], losses[0])
        self.assertFalse(np.array_equal(before['w1'], network.parameters['w1']))

    def test_export_matches_inference(self):
        network = Network(np.random.default_rng(42))
        inputs = np.random.default_rng(1).normal(size=32).astype(np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.json'
            network.export(path)
            loaded = Network.load(path)
        np.testing.assert_array_equal(network.predict(inputs), loaded.predict(inputs))

    def test_bellman_target_masks_invalid_actions(self):
        target = Network(np.random.default_rng(0))
        target.parameters['w1'].fill(0)
        target.parameters['w2'].fill(0)
        target.parameters['b2'][:] = [1, 10000, 10000, 10000, 2]
        replay = ReplayBuffer(2)
        replay.append(np.zeros(32), 0, 1, np.zeros(32),
                      np.array([True, False, False, False, True]), False, 1)

        class RecordingNetwork:
            def update(self, inputs, actions, targets):
                np.testing.assert_allclose(targets, 1 + 0.995 * 2)
                return 0

        replay.train_batch(RecordingNetwork(), target, np.random.default_rng(0), 1)


if __name__ == '__main__':
    unittest.main()
