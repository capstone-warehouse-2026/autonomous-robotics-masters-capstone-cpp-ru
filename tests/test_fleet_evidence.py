"""Проверки независимого аудитора флота без запуска ROS и планировщика."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_evidence import audit_assignments  # noqa: E402
from fleet_evidence import audit_jobs, audit_plans, audit_rounds  # noqa: E402
from fleet_evidence import count_path_conflicts, segments_distance  # noqa: E402
import yaml  # noqa: E402


def event(action, stamp, robot='', job='', detail=''):
    return {'event': action, 'sim_time': str(stamp), 'robot_id': robot,
            'job_id': job, 'detail': detail}


class MapFixture:
    def __init__(self, occupied=None):
        self.occupied = occupied

    def clearance(self, point):
        if self.occupied and tuple(point) == self.occupied:
            return 0.0
        return 2.0


class GeometryTests(unittest.TestCase):
    def test_crossing_swap_and_following_are_conflicts(self):
        self.assertEqual(segments_distance((0, 1), (2, 1), (1, 0), (1, 2)), 0.0)
        self.assertGreater(count_path_conflicts([
            [(0, 0), (1, 0)], [(1, 0), (0, 0)]]), 0)
        self.assertGreater(count_path_conflicts([
            [(0, 0), (1, 0)], [(1, 0), (2, 0)]]), 0)

    def test_parallel_edges_and_waits_keep_clearance(self):
        self.assertEqual(count_path_conflicts([
            [(0, 0), (1, 0)], [(0, 1), (1, 1)]]), 0)
        self.assertEqual(count_path_conflicts([[(0, 0)], [(1, 0)]]), 0)

    def test_goal_remains_occupied_after_short_path_ends(self):
        self.assertGreater(count_path_conflicts([
            [(1, 0)], [(3, 0), (2, 0), (1, 0)]]), 0)


class LedgerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.runtime = Path(self.directory.name)
        self.scenario = {
            'robots': [{'id': 'robot_1'}, {'id': 'robot_2'}],
            'stations': [{'id': 'left', 'x': -1.0, 'y': 0.0},
                         {'id': 'right', 'x': 1.0, 'y': 0.0}],
            'jobs': [
                {'id': 'a', 'pickup': 'left', 'dropoff': 'right', 'release_time': 0.0,
                 'pickup_service': 1.0, 'dropoff_service': 1.0},
                {'id': 'b', 'pickup': 'right', 'dropoff': 'left', 'release_time': 0.0,
                 'pickup_service': 1.0, 'dropoff_service': 1.0},
            ],
        }
        self.events = [
            event('fleet_started', 0.0), event('assigned', 0.0, 'robot_1', 'a'),
            event('assigned', 0.0, 'robot_2', 'b'),
            event('pickup_service', 0.0, 'robot_1', 'a'),
            event('pickup_service', 0.0, 'robot_2', 'b'),
            event('picked', 1.0, 'robot_1', 'a'), event('picked', 1.0, 'robot_2', 'b'),
            event('dropoff_service', 4.0, 'robot_1', 'a'),
            event('dropoff_service', 4.0, 'robot_2', 'b'),
            event('delivered', 5.0, 'robot_1', 'a'),
            event('delivered', 5.0, 'robot_2', 'b'),
        ]
        self.samples = {'robot_1': [], 'robot_2': []}
        for tick in range(51):
            stamp = tick / 10
            x = -1.0 + 2.0 * min(1.0, max(0.0, (stamp - 1.0) / 3.0))
            speed = 2.0 / 3.0 if 1.0 < stamp < 4.0 else 0.0
            self.samples['robot_1'].append((stamp, x, 0.0, 0.0, speed, 0.0))
            self.samples['robot_2'].append((stamp, -x, 0.0, 0.0, speed, 0.0))

    def write_events(self):
        with (self.runtime / 'events.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, ['event', 'sim_time', 'robot_id', 'job_id', 'detail'])
            writer.writeheader()
            writer.writerows(self.events)

    def audit(self):
        self.write_events()
        return audit_jobs(self.runtime, self.scenario, self.samples, completed=True)

    def test_valid_unique_ownership_and_station_services(self):
        checks, _ = self.audit()
        self.assertEqual(len(checks), 4)
        self.assertTrue(all(check['position_error_m'] < 1e-8 for check in checks))

    def test_duplicate_job_assignment_is_rejected(self):
        self.events.insert(3, event('assigned', 0.0, 'robot_2', 'a'))
        with self.assertRaises(ValueError):
            self.audit()

    def test_robot_cannot_take_second_job_while_loaded(self):
        self.events[2]['robot_id'] = 'robot_1'
        with self.assertRaises(ValueError):
            self.audit()

    def test_pickup_by_different_owner_is_rejected(self):
        next(row for row in self.events if row['event'] == 'picked')['robot_id'] = 'robot_2'
        with self.assertRaises(ValueError):
            self.audit()

    def test_wrong_physical_station_is_rejected(self):
        self.samples['robot_1'][-1] = (5.0, 0.5, 0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            self.audit()

    def test_missing_ground_truth_at_service_is_rejected(self):
        self.samples['robot_1'] = self.samples['robot_1'][:-3]
        with self.assertRaises(ValueError):
            self.audit()

    def test_assignment_before_release_is_rejected(self):
        self.scenario['jobs'][0]['release_time'] = 2.0
        with self.assertRaises(ValueError):
            self.audit()

    def test_nonfinite_event_time_is_rejected(self):
        self.events[1]['sim_time'] = 'nan'
        with self.assertRaises(ValueError):
            self.audit()

    def test_service_while_moving_is_rejected(self):
        self.samples['robot_1'][10] = (1.0, -1.0, 0.0, 0.0, 0.35, 0.0)
        with self.assertRaises(ValueError):
            self.audit()

    def test_missing_service_start_is_rejected(self):
        self.events = [row for row in self.events if row['event'] != 'pickup_service']
        with self.assertRaises(ValueError):
            self.audit()

    def test_configured_service_duration_is_observed(self):
        for row in self.events:
            if row['event'] == 'dropoff_service':
                row['sim_time'] = '4.8'
        with self.assertRaises(ValueError):
            self.audit()

    def test_movement_inside_service_window_is_rejected(self):
        self.samples['robot_1'][45] = (4.5, 1.0, 0.0, 0.0, 0.35, 0.0)
        with self.assertRaises(ValueError):
            self.audit()


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.runtime = Path(self.directory.name)

    def write_paths(self, variants):
        with (self.runtime / 'plans.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, ['plan_id', 'variant', 'robot_id', 'step', 'x', 'y'])
            writer.writeheader()
            for variant, paths in variants.items():
                for robot, path in paths.items():
                    for step, point in enumerate(path):
                        writer.writerow({'plan_id': '0', 'variant': variant, 'robot_id': robot,
                                         'step': step, 'x': point[0], 'y': point[1]})

    def test_resolved_path_cannot_change_robot_goal(self):
        self.write_paths({
            'independent': {'robot_1': [(0, 0), (1, 0)], 'robot_2': [(2, 0), (1, 0)]},
            'resolved': {'robot_1': [(0, 0), (1, 0)], 'robot_2': [(2, 1), (1, 1)]},
        })
        with self.assertRaises(ValueError):
            audit_plans(self.runtime, ['robot_1', 'robot_2'], MapFixture())

    def test_wait_only_path_still_needs_static_clearance(self):
        paths = {'robot_1': [(2, 2)], 'robot_2': [(4, 2)]}
        self.write_paths({'independent': paths, 'resolved': paths})
        with self.assertRaises(ValueError):
            audit_plans(self.runtime, ['robot_1', 'robot_2'], MapFixture((2, 2)))

    def test_fractional_transition_is_not_a_unit_grid_edge(self):
        paths = {'robot_1': [(0, 0), (0.5, 0)], 'robot_2': [(0, 2), (0.5, 2)]}
        self.write_paths({'independent': paths, 'resolved': paths})
        with self.assertRaises(ValueError):
            audit_plans(self.runtime, ['robot_1', 'robot_2'], MapFixture())


class AssignmentTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.runtime = Path(self.directory.name)
        self.data = {
            'robot_ids': ['robot_1', 'robot_2'], 'job_ids': ['a', 'b'],
            'cost_matrix': [[1, 2], [2, 100]], 'total_cost': 4,
            'chosen_pairs': [{'robot_id': 'robot_1', 'job_id': 'b', 'cost': 2},
                             {'robot_id': 'robot_2', 'job_id': 'a', 'cost': 2}],
        }

    def audit(self):
        (self.runtime / 'assignment_1.yaml').write_text(yaml.safe_dump(self.data))
        return audit_assignments(self.runtime)

    def test_minimum_cost_matching_passes_independent_oracle(self):
        reports = self.audit()
        self.assertEqual(reports[0]['assigned'], 2)
        self.assertEqual(reports[0]['optimal_cost'], 4)

    def test_locally_greedy_nonoptimal_matching_is_rejected(self):
        self.data['chosen_pairs'] = [
            {'robot_id': 'robot_1', 'job_id': 'a', 'cost': 1},
            {'robot_id': 'robot_2', 'job_id': 'b', 'cost': 100}]
        self.data['total_cost'] = 101
        with self.assertRaises(ValueError):
            self.audit()

    def test_duplicate_owner_is_rejected(self):
        self.data['chosen_pairs'][1]['robot_id'] = 'robot_1'
        with self.assertRaises(ValueError):
            self.audit()

    def test_incomplete_matching_is_rejected_when_more_jobs_can_be_assigned(self):
        self.data['chosen_pairs'] = self.data['chosen_pairs'][:1]
        self.data['total_cost'] = 2
        with self.assertRaises(ValueError):
            self.audit()

    def test_incorrect_recorded_cost_is_rejected(self):
        self.data['chosen_pairs'][0]['cost'] = 3
        with self.assertRaises(ValueError):
            self.audit()

    def test_forbidden_pickup_assignment_is_rejected(self):
        self.data['cost_matrix'][0][1] = -1
        with self.assertRaises(ValueError):
            self.audit()


class RoundTests(unittest.TestCase):
    def setUp(self):
        self.paths = {'0': {'robot_1': [(0, 0), (1, 0), (2, 0)],
                            'robot_2': [(0, 1), (1, 1), (2, 1)]}}
        self.events = [event('round_started', 0.0, detail='plan=0;step=1'),
                       event('round_completed', 4.0, detail='plan=0;step=1'),
                       event('round_started', 4.5, detail='plan=0;step=2'),
                       event('round_completed', 8.5, detail='plan=0;step=2')]
        self.samples = {'robot_1': [], 'robot_2': []}
        for tick in range(426):
            stamp = tick / 50
            if stamp < 4.5:
                x = min(1.0, max(0.0, (stamp - 0.3) / 3.0))
                moving = 0.3 < stamp < 3.3
            else:
                x = 1.0 + min(1.0, max(0.0, (stamp - 4.8) / 3.0))
                moving = 4.8 < stamp < 7.8
            for robot, y in [('robot_1', 0.0), ('robot_2', 1.0)]:
                self.samples[robot].append((stamp, x, y, 0.0, 1 / 3 if moving else 0.0, 0.0))

    def test_valid_synchronized_rounds_follow_straight_edges(self):
        report = audit_rounds(self.events, self.paths, self.samples)
        self.assertEqual(report['verified_rounds'], 2)
        self.assertLess(report['max_edge_deviation_m'], 1e-8)

    def test_no_round_evidence_is_rejected(self):
        with self.assertRaises(ValueError):
            audit_rounds([], self.paths, self.samples)

    def test_missing_one_robot_measurements_is_rejected(self):
        self.samples['robot_2'] = []
        with self.assertRaises(ValueError):
            audit_rounds(self.events, self.paths, self.samples)

    def test_missing_mid_round_measurements_is_rejected(self):
        self.samples['robot_1'] = [sample for sample in self.samples['robot_1']
                                   if not 1.2 <= sample[0] <= 2.8]
        with self.assertRaises(ValueError):
            audit_rounds(self.events, self.paths, self.samples)

    def test_departure_from_reserved_corridor_is_rejected(self):
        self.samples['robot_1'][100] = (2.0, 0.5, 0.3, 0.0, 0.2, 0.0)
        with self.assertRaises(ValueError):
            audit_rounds(self.events, self.paths, self.samples)

    def test_unfinished_final_round_is_not_silent_success(self):
        with self.assertRaises(ValueError):
            audit_rounds(self.events[:-1], self.paths, self.samples)

    def test_cancelled_partial_round_is_audited_without_claiming_completion(self):
        for robot, trajectory in self.samples.items():
            stopped = []
            for sample in trajectory:
                if sample[0] > 6.0:
                    continue
                if sample[0] >= 5.4:
                    sample = (sample[0], 1.2, sample[2], 0.0, 0.0, 0.0)
                stopped.append(sample)
            self.samples[robot] = stopped
        report = audit_rounds(self.events[:-1], self.paths, self.samples,
                              allow_interrupted=True)
        self.assertEqual(report['completed_rounds'], 1)
        self.assertEqual(report['interrupted_rounds'], 1)

    def test_round_cannot_complete_before_target_is_reached(self):
        early = [self.events[0], event('round_completed', 1.0, detail='plan=0;step=1')]
        short_paths = {'0': {robot: path[:2] for robot, path in self.paths['0'].items()}}
        with self.assertRaises(ValueError):
            audit_rounds(early, short_paths, self.samples)

    def test_skipped_round_is_rejected(self):
        with self.assertRaises(ValueError):
            audit_rounds(self.events[2:], self.paths, self.samples)


if __name__ == '__main__':
    unittest.main()
