"""CLI-проверка пересчёта ROS ledger: валидные эпизоды и отклонение повреждённых."""

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

EXECUTABLE = sys.argv.pop(1)
HEADER = ['episode_id', 'job_id', 'sim_time', 'event', 'reason', 'target']


class MissionMetricsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.scenario = {
            'episode_id': 'scenario', 'family': 'low', 'initial_station': 's0', 'horizon': 30,
            'stations': [{'id': 's0', 'x': 0, 'y': 0}, {'id': 's1', 'x': 1, 'y': 0}],
            'jobs': [{'id': 'a', 'pickup': 's0', 'dropoff': 's1', 'release_time': 0,
                      'due_time': 10},
                     {'id': 'b', 'pickup': 's1', 'dropoff': 's0', 'release_time': 5,
                      'due_time': 20}],
        }
        self.runner = {'episode_id': 'e1', 'method': 'learned', 'terminal_reason': 'completed',
                       'elapsed_sim_s': 11, 'distance_m': 4}
        self.events = [
            ['', 0, 'episode_started', ''], ['a', 0, 'released', ''],
            ['', 0.5, 'decision', 'latency_s=0.002'], ['a', 1, 'selected', ''],
            ['a', 2, 'picked', ''], ['a', 4, 'delivered', ''],
            ['b', 5, 'released', ''], ['', 5.5, 'decision', 'latency_s=0.003'],
            ['b', 6, 'selected', ''], ['b', 7, 'picked', ''],
            ['b', 9, 'delivered', ''], ['', 10, 'episode_completed', ''],
        ]

    def run_metrics(self, success=True, other_episode=False):
        (self.root / 'scenario.json').write_text(json.dumps(self.scenario))
        (self.root / 'runner.json').write_text(json.dumps(self.runner))
        with (self.root / 'events.csv').open('w', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(HEADER)
            for job, stamp, event, reason in self.events:
                writer.writerow(['e1', job, stamp, event, reason, ''])
            if other_episode:
                writer.writerow(['e2', 'unknown', 99, 'delivered', '', ''])
        result = subprocess.run(
            [EXECUTABLE, str(self.root / 'scenario.json'), str(self.root / 'events.csv'),
             str(self.root / 'runner.json'), str(self.root / 'metrics.csv')],
            capture_output=True, text=True, timeout=10)
        if not success:
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertFalse((self.root / 'metrics.csv').exists())
            return None
        self.assertEqual(result.returncode, 0, result.stderr)
        with (self.root / 'metrics.csv').open() as stream:
            return next(csv.DictReader(stream))

    def test_exact_episode_and_empty_queue_idle(self):
        row = self.run_metrics(other_episode=True)
        self.assertEqual(row['episode_id'], 'e1')
        self.assertEqual(row['delivered'], '2')
        self.assertEqual(row['released'], '2')
        self.assertEqual(float(row['idle_s']), 4)
        self.assertEqual(float(row['elapsed_sim_s']), 10)
        self.assertEqual(float(row['decision_latency_p95_ms']), 3)

    def test_unknown_job_rejected(self):
        self.events[3][0] = 'unknown'
        self.run_metrics(False)

    def test_delivered_without_pickup_rejected(self):
        self.events.pop(4)
        self.run_metrics(False)

    def test_delivery_after_failure_rejected(self):
        self.events.insert(5, ['a', 3, 'failed', 'navigation_failure'])
        self.run_metrics(False)

    def test_overlapping_jobs_rejected(self):
        self.scenario['jobs'][1]['release_time'] = 0
        self.events.insert(5, ['b', 3, 'released', ''])
        self.events.insert(6, ['b', 3.5, 'selected', ''])
        self.run_metrics(False)

    def test_duplicate_pickup_rejected(self):
        self.events.insert(5, ['a', 3, 'picked', ''])
        self.run_metrics(False)

    def test_time_regression_rejected(self):
        self.events[5][1] = 1.5
        self.run_metrics(False)

    def test_negative_or_nonfinite_inputs_rejected(self):
        self.runner['distance_m'] = -1
        self.run_metrics(False)

    def test_nonfinite_latency_rejected(self):
        self.events[2][3] = 'latency_s=nan'
        self.run_metrics(False)

    def test_partial_latency_report_is_na(self):
        self.events[2][3] = ''
        row = self.run_metrics()
        self.assertEqual(row['decision_latency_p95_ms'], 'NA')

    def test_release_before_manifest_time_rejected(self):
        self.scenario['jobs'][0]['release_time'] = 1
        self.run_metrics(False)

    def test_runner_completion_requires_completed_ledger(self):
        self.events[-1][2] = 'episode_cancelled'
        self.run_metrics(False)

    def test_loaded_failure_blocks_following_selection(self):
        self.events[5][2] = 'failed'
        self.runner['terminal_reason'] = 'mission_fault'
        self.events[-1][2] = 'episode_fault'
        self.run_metrics(False)

    def test_unfinished_deliveries_cannot_report_complete(self):
        self.events = self.events[:6] + [['', 10, 'episode_completed', '']]
        self.run_metrics(False)

    def test_no_deliveries_distance_per_job_is_na(self):
        self.runner['terminal_reason'] = 'mission_cancelled'
        self.events = self.events[:4] + [
            ['a', 2, 'failed', 'cancelled'], ['', 3, 'episode_cancelled', '']]
        row = self.run_metrics()
        self.assertEqual(row['delivered'], '0')
        self.assertEqual(row['distance_per_job_m'], 'NA')
        self.assertEqual(row['tardiness_mean_s'], 'NA')
        self.assertEqual(row['failed'], '1')

    def test_recorded_ros_ledgers(self):
        repository = Path(__file__).resolve().parents[4]
        fixture = repository / 'experiments/g5/ros-demo'
        for mode in ('classical', 'learned', 'blocked', 'controller_failure', 'frozen'):
            with self.subTest(mode=mode):
                directory = fixture / mode
                scenario = repository / 'ros2_ws/src/capstone_mission/config/demo.yaml'
                if mode == 'frozen':
                    scenario = directory / 'scenario.yaml'
                output = self.root / f'{mode}.csv'
                result = subprocess.run(
                    [EXECUTABLE, str(scenario), str(directory / 'events.csv'),
                     str(directory / 'episode.yaml'), str(output)],
                    capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr)
                with output.open() as stream:
                    actual = next(csv.DictReader(stream))
                with (directory / 'metrics.csv').open() as stream:
                    expected = next(csv.DictReader(stream))
                for key in ('episode_id', 'released', 'delivered', 'on_time', 'unfinished',
                            'failed', 'distance_m', 'idle_s'):
                    self.assertEqual(actual[key], expected[key], key)


if __name__ == '__main__':
    unittest.main()
