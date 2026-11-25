"""Граница прогрева и физического контроля внешнего ROS-аудитора."""
from pathlib import Path
import sys
import time
from types import MethodType, SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
try:
    from evaluate_fleet import FleetEvidence
except ModuleNotFoundError:
    FleetEvidence = None


@unittest.skipIf(FleetEvidence is None, 'Нужно source /opt/ros/jazzy/setup.bash')
class ObserverWarmupTests(unittest.TestCase):
    def setUp(self):
        self.results = []
        now = time.monotonic()
        self.observer = SimpleNamespace(
            finished=False, failure=None, ready=True, monitoring_armed=None,
            robot_ids=['robot_1', 'robot_2'],
            contact_received={'robot_1': now - 1, 'robot_2': now - 1},
            started_wall=now, args=SimpleNamespace(timeout=60, mode='delivery'),
            cancel_future=None, terminal=None,
            finish=lambda success, reason: self.results.append((success, reason)))
        self.observer.arm_monitoring = MethodType(FleetEvidence.arm_monitoring, self.observer)

    def test_stationary_warmup_is_bounded_without_motion_watchdog(self):
        FleetEvidence.on_tick(self.observer)
        self.assertEqual(self.results, [])
        self.observer.started_wall -= 61
        FleetEvidence.on_tick(self.observer)
        self.assertEqual(self.results, [(False, 'wall_timeout')])

    def test_running_status_arms_watchdog_before_first_motion(self):
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
            status=[SimpleNamespace(name='capstone_fleet/mission', message='running')])
        FleetEvidence.on_status(self.observer, message)
        FleetEvidence.on_tick(self.observer)
        self.assertEqual(self.results, [(False, 'missing_physical_contact_samples')])

    def test_motion_without_mission_status_also_arms_watchdog(self):
        self.observer.arm_monitoring('physical_motion', 10)
        FleetEvidence.on_tick(self.observer)
        self.assertEqual(self.results, [(False, 'missing_physical_contact_samples')])

    def test_monitoring_latches_first_reason_and_time(self):
        self.observer.arm_monitoring('physical_motion', 10)
        self.observer.arm_monitoring('mission_running', 11)
        self.assertEqual(self.observer.monitoring_armed['reason'], 'physical_motion')
        self.assertEqual(self.observer.monitoring_armed['sim_time'], 10)


if __name__ == '__main__':
    unittest.main()
