#!/usr/bin/env python3
"""Сверить совместные планы и доставки с физической истиной Webots."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import itertools
import math
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from fleet_evidence import audit_assignments, audit_jobs, audit_plans, audit_rounds, StaticMap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import yaml


class FleetEvidence(Node):
    def __init__(self, args):
        super().__init__('fleet_evaluator', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.args = args
        self.scenario = yaml.safe_load(args.manifest.read_text())
        self.robot_ids = [robot['id'] for robot in self.scenario['robots']]
        self.static_map = StaticMap(args.map)
        args.output.mkdir(parents=True, exist_ok=False)
        self.stream = (args.output / 'trajectory.csv').open('w', newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['robot_id', 'source', 'sim_time', 'x', 'y', 'yaw', 'v', 'omega'])
        self.contact_stream = (args.output / 'contacts.csv').open('w', newline='')
        self.contact_writer = csv.writer(self.contact_stream)
        self.contact_writer.writerow(['robot_id', 'sim_time', 'contact'])
        self.samples = {robot: [] for robot in self.robot_ids}
        self.truth = {}
        self.odometry = {}
        self.distance = {robot: 0.0 for robot in self.robot_ids}
        self.contacts = {robot: 0 for robot in self.robot_ids}
        self.contact_counts = {robot: 0 for robot in self.robot_ids}
        self.contact_received = {}
        self.maximum_contact_wall_gap = 0.0
        self.maximum_truth_sim_gap = 0.0
        self.maximum_pair_sim_gap = 0.0
        self.last_pair_time = None
        self.stopped_since = {}
        self.synchronized = {}
        self.pair_samples = 0
        self.minimum_separation = math.inf
        self.minimum_map_clearance = math.inf
        self.started_wall = time.monotonic()
        self.terminal = None
        self.terminal_stamp = None
        self.terminal_received = None
        self.failure = None
        self.finished = False
        self.exit_code = 1
        self.ready = False
        self.ready_since = None
        self.monitoring_armed = None
        self.audit_worker = ThreadPoolExecutor(max_workers=1)
        self.audit_future = None
        self.cancel_future = None
        self.cancel_accepted = False
        self.cancel_requested = None
        self.subscriptions_kept = []
        self.create_interfaces()
        self.timer = self.create_timer(0.05, self.on_tick, clock=rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def create_interfaces(self):
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        for robot in self.robot_ids:
            self.subscriptions_kept.extend([
                self.create_subscription(Odometry, f'/{robot}/evaluation/ground_truth',
                                         lambda message, name=robot: self.on_truth(name, message),
                                         qos),
                self.create_subscription(Odometry, f'/{robot}/state/odom',
                                         lambda msg, name=robot: self.on_odometry(name, msg),
                                         qos),
                self.create_subscription(Bool, f'/{robot}/evaluation/contact',
                                         lambda msg, name=robot: self.on_contact(name, msg),
                                         qos),
            ])
        self.status_subscription = self.create_subscription(
            DiagnosticArray, 'fleet/status', self.on_status, qos)
        self.cancel_client = self.create_client(Trigger, 'fleet/cancel')

    def sample(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = message.pose.pose
        rotation = pose.orientation
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        velocity = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        result = (stamp, pose.position.x, pose.position.y, yaw,
                  velocity, message.twist.twist.angular.z)
        if not all(map(math.isfinite, result)):
            self.failure = 'nonfinite_physical_measurement'
            return None
        return result

    def on_truth(self, robot, message):
        if message.header.frame_id != 'map':
            self.failure = 'ground_truth_frame_is_not_map'
            return
        sample = self.sample(message)
        if sample is None:
            return
        previous = self.truth.get(robot)
        observed_speed = 0.0
        if previous is not None:
            elapsed = sample[0] - previous[0][0]
            if elapsed <= 0:
                self.failure = 'unexpected_physical_clock_reset_or_duplicate'
                return
            if self.ready:
                self.maximum_truth_sim_gap = max(self.maximum_truth_sim_gap, elapsed)
                if elapsed > 0.060001:
                    self.failure = 'missing_physical_truth_samples'
            travelled = math.dist(sample[1:3], previous[0][1:3])
            observed_speed = travelled / elapsed
            if observed_speed > 0.9:
                self.failure = 'physical_teleport_or_speed_limit_violation'
            self.distance[robot] += travelled
        if (self.distance[robot] > 0.01 or max(sample[4], observed_speed) >= 0.03 or
                abs(sample[5]) >= 0.08):
            self.arm_monitoring('physical_motion', sample[0])
        self.truth[robot] = (sample, time.monotonic())
        self.samples[robot].append(sample)
        self.writer.writerow([robot, 'ground_truth', *sample])
        clearance = self.static_map.clearance(sample[1:3])
        self.minimum_map_clearance = min(self.minimum_map_clearance, clearance)
        if clearance < 0.279:
            self.failure = 'physical_robot_intersects_static_map'
        if max(sample[4], observed_speed) < 0.03 and abs(sample[5]) < 0.08:
            self.stopped_since.setdefault(robot, sample[0])
        else:
            self.stopped_since.pop(robot, None)
        stamp_key = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        positions = self.synchronized.setdefault(stamp_key, {})
        positions[robot] = sample[1:3]
        if len(positions) == len(self.robot_ids):
            if self.ready and self.last_pair_time is not None:
                gap = sample[0] - self.last_pair_time
                self.maximum_pair_sim_gap = max(self.maximum_pair_sim_gap, gap)
                if gap > 0.060001 or gap <= 0:
                    self.failure = 'missing_synchronized_robot_samples'
            self.last_pair_time = sample[0]
            for first, second in itertools.combinations(positions.values(), 2):
                separation = math.dist(first, second)
                self.minimum_separation = min(self.minimum_separation, separation)
                if separation < 0.60:
                    self.failure = 'insufficient_physical_robot_separation'
            self.pair_samples += 1
            del self.synchronized[stamp_key]
        # Потеря измерений не превращается в неограниченную очередь.
        for key in list(self.synchronized):
            if stamp_key - key > 2000000000:
                del self.synchronized[key]

    def on_odometry(self, robot, message):
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            self.failure = 'invalid_encoder_odometry_frames'
            return
        sample = self.sample(message)
        if sample is not None:
            self.odometry[robot] = (sample, time.monotonic())
            self.writer.writerow([robot, 'wheel_odom', *sample])

    def on_contact(self, robot, message):
        self.contact_counts[robot] += 1
        received = time.monotonic()
        if self.ready and robot in self.contact_received:
            self.maximum_contact_wall_gap = max(
                self.maximum_contact_wall_gap, received - self.contact_received[robot])
        self.contact_received[robot] = received
        self.contacts[robot] += bool(message.data)
        self.contact_writer.writerow([robot, self.get_clock().now().nanoseconds * 1e-9,
                                      int(message.data)])
        if message.data:
            self.failure = 'physical_collision_contact'

    def on_status(self, message):
        for status in message.status:
            if status.name != 'capstone_fleet/mission':
                continue
            if status.message == 'running':
                self.arm_monitoring('mission_running', message.header.stamp.sec +
                                    message.header.stamp.nanosec * 1e-9)
            if status.message in ['completed', 'cancelled', 'fault'] and self.terminal is None:
                self.terminal = status.message
                self.terminal_stamp = (message.header.stamp.sec +
                                       message.header.stamp.nanosec * 1e-9)
                self.terminal_received = time.monotonic()

    def arm_monitoring(self, reason, stamp):
        # Однажды включённый watchdog действует и при ожидании, и после отмены.
        if self.monitoring_armed is None:
            self.monitoring_armed = {
                'reason': reason, 'sim_time': stamp,
                'wall_seconds_since_observer_start': time.monotonic() - self.started_wall,
            }

    def request_cancel(self):
        if not self.cancel_client.service_is_ready():
            return
        self.cancel_requested = time.monotonic()
        self.cancel_future = self.cancel_client.call_async(Trigger.Request())
        self.cancel_future.add_done_callback(self.on_cancel_response)

    def on_cancel_response(self, future):
        try:
            self.cancel_accepted = future.result().success
            if not self.cancel_accepted:
                self.failure = 'fleet_cancel_rejected'
        except Exception as error:
            self.failure = f'fleet_cancel_error: {error}'

    def physical_stop_confirmed(self):
        for robot in self.robot_ids:
            if robot not in self.truth or robot not in self.odometry:
                return False
            truth, received = self.truth[robot]
            odometry, odom_received = self.odometry[robot]
            if max(time.monotonic() - received, time.monotonic() - odom_received,
                   time.monotonic() - self.contact_received.get(robot, 0)) > 0.3:
                return False
            if odometry[4] >= 0.03 or abs(odometry[5]) >= 0.08:
                return False
            if robot not in self.stopped_since:
                return False
            if truth[0] - max(self.terminal_stamp, self.stopped_since[robot]) < 0.3:
                return False
        return True

    def on_tick(self):
        if self.finished:
            return
        if self.failure:
            self.finish(False, self.failure)
            return
        # До готовности Nav2/координатора физика может ждать подключения DDS.
        # После старта (или любого фактического движения) пропуск уже не допустим.
        if self.monitoring_armed is not None and any(
                time.monotonic() - self.contact_received.get(robot, 0) > 0.3
                for robot in self.robot_ids):
            self.finish(False, 'missing_physical_contact_samples')
            return
        if time.monotonic() - self.started_wall > self.args.timeout:
            self.finish(False, 'wall_timeout')
            return
        if not self.ready:
            fresh = all(robot in self.truth and robot in self.odometry and
                        max(time.monotonic() - self.truth[robot][1],
                            time.monotonic() - self.odometry[robot][1],
                            time.monotonic() - self.contact_received.get(robot, 0)) < 0.3
                        for robot in self.robot_ids)
            if not fresh:
                self.ready_since = None
            elif self.ready_since is None:
                self.ready_since = time.monotonic()
            elif time.monotonic() - self.ready_since >= 1.0:
                (self.args.output / 'ready').write_text('physical subscriptions receiving data\n')
                self.ready = True
        if self.args.mode == 'cancel' and self.cancel_requested is None and self.terminal is None:
            if max(self.distance.values()) >= 1.5:
                self.request_cancel()
        if self.cancel_future is not None and not self.cancel_future.done():
            if time.monotonic() - self.cancel_requested > 3:
                self.finish(False, 'cancel_response_timeout')
                return
        if self.terminal is None:
            return
        expected = 'completed' if self.args.mode == 'delivery' else 'cancelled'
        if self.terminal != expected:
            self.finish(False, 'unexpected_terminal_' + self.terminal)
            return
        if not self.physical_stop_confirmed():
            if time.monotonic() - self.terminal_received > 8:
                self.finish(False, 'physical_stop_not_confirmed')
            return
        if self.audit_future is None:
            snapshot = {robot: list(values) for robot, values in self.samples.items()}
            self.audit_future = self.audit_worker.submit(self.audit_recorded_execution, snapshot)
            return
        if not self.audit_future.done():
            return
        try:
            details = self.audit_future.result()
            self.finish(True, 'verified', details)
        except (ValueError, KeyError, OSError) as error:
            self.finish(False, str(error))

    def audit_recorded_execution(self, samples):
        moved = all(distance >= 1.0 for distance in self.distance.values())
        if not moved or self.pair_samples < 50:
            raise ValueError('insufficient_multi_robot_physical_motion_evidence')
        reports, paths = audit_plans(self.args.runtime_output, self.robot_ids, self.static_map)
        if not any(report['initial_conflicts'] > 0 for report in reports):
            raise ValueError('scenario_did_not_exercise_route_conflicts')
        positions, events = audit_jobs(
            self.args.runtime_output, self.scenario, samples,
            self.terminal == 'completed')
        rounds = audit_rounds(events, paths, samples,
                              allow_interrupted=self.terminal == 'cancelled')
        if self.args.mode == 'cancel' and not self.cancel_accepted:
            raise ValueError('cancellation_not_acknowledged')
        return {'plans': reports, 'service_positions': positions, 'execution': rounds,
                'assignments': audit_assignments(self.args.runtime_output)}

    def finish(self, success, reason, details=None):
        result = {
            'success': success, 'reason': reason,
            'scope': 'physical_multi_robot_Webots_CBS_ECBS_dispatch',
            'terminal': self.terminal, 'robots': self.robot_ids,
            'monitoring_armed': self.monitoring_armed,
            'ground_truth_samples': {robot: len(values) for robot, values in self.samples.items()},
            'distance_m': self.distance, 'contact_samples': self.contact_counts,
            'physical_contacts': self.contacts, 'synchronized_pair_samples': self.pair_samples,
            'minimum_robot_separation_m': self.minimum_separation,
            'minimum_static_clearance_m': self.minimum_map_clearance,
            'maximum_truth_sim_gap_s': self.maximum_truth_sim_gap,
            'maximum_pair_sim_gap_s': self.maximum_pair_sim_gap,
            'maximum_contact_wall_gap_s': self.maximum_contact_wall_gap,
            'physical_stop_confirmed': bool(self.terminal and self.physical_stop_confirmed()),
        }
        result.update(details or {})
        self.stream.flush()
        self.contact_stream.flush()
        (self.args.output / 'audit.yaml').write_text(yaml.safe_dump(result, sort_keys=False))
        self.exit_code = 0 if success else 1
        self.finished = True
        self.get_logger().info(f'Physical fleet audit: {reason}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--runtime-output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=480)
    parser.add_argument('--mode', choices=['delivery', 'cancel'], default='delivery')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = FleetEvidence(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code
    except KeyboardInterrupt:
        return 1
    finally:
        node.stream.close()
        node.contact_stream.close()
        node.audit_worker.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
