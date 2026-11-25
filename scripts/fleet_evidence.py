"""Независимые проверки журналов флота; код планировщика здесь не используется."""
import csv
import itertools
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def point_segment_distance(point, start, end):
    vector = (end[0] - start[0], end[1] - start[1])
    length_squared = vector[0] ** 2 + vector[1] ** 2
    if length_squared == 0:
        return math.dist(point, start)
    fraction = sum((point[i] - start[i]) * vector[i] for i in range(2)) / length_squared
    fraction = max(0, min(1, fraction))
    return math.dist(point, [start[i] + fraction * vector[i] for i in range(2)])


def segments_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    if (cross(a, b, c) * cross(a, b, d) < 0 and
            cross(c, d, a) * cross(c, d, b) < 0):
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def count_path_conflicts(paths, clearance=0.85):
    if not paths or any(not path for path in paths):
        raise ValueError('missing robot paths')
    horizon = max(map(len, paths))
    count = 0
    for first, second in itertools.combinations(paths, 2):
        for step in range(max(1, horizon - 1)):
            a, b = first[min(step, len(first) - 1)], first[min(step + 1, len(first) - 1)]
            c, d = second[min(step, len(second) - 1)], second[min(step + 1, len(second) - 1)]
            if segments_distance(a, b, c, d) < clearance - 1e-8:
                count += 1
    return count


class StaticMap:
    def __init__(self, path):
        path = Path(path)
        data = yaml.safe_load(path.read_text())
        if abs(data['origin'][2]) > 1e-8 or data.get('negate', 0):
            raise ValueError('evaluator expects axis-aligned, non-negated map')
        pixels = np.asarray(Image.open(path.parent / data['image']).convert('L'))
        # Неизвестные пиксели тоже нельзя считать доказанно свободными.
        blocked = pixels < 250
        rows, columns = np.nonzero(blocked)
        self.resolution = float(data['resolution'])
        self.x = data['origin'][0] + (columns + 0.5) * self.resolution
        self.y = data['origin'][1] + (pixels.shape[0] - rows - 0.5) * self.resolution
        self.bounds = (data['origin'][0], data['origin'][1],
                       data['origin'][0] + pixels.shape[1] * self.resolution,
                       data['origin'][1] + pixels.shape[0] * self.resolution)

    def clearance(self, point):
        half = self.resolution / 2
        dx = np.maximum(np.abs(self.x - point[0]) - half, 0)
        dy = np.maximum(np.abs(self.y - point[1]) - half, 0)
        walls = (point[0] - self.bounds[0], point[1] - self.bounds[1],
                 self.bounds[2] - point[0], self.bounds[3] - point[1])
        return min(float(np.min(np.hypot(dx, dy), initial=math.inf)), *walls)


def read_rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def audit_plans(runtime, robot_ids, static_map):
    groups = {}
    for row in read_rows(Path(runtime) / 'plans.csv'):
        point = (float(row['x']), float(row['y']))
        if not all(map(math.isfinite, point)):
            raise ValueError('nonfinite planned point')
        group = groups.setdefault(row['plan_id'], {}).setdefault(row['variant'], {})
        path = group.setdefault(row['robot_id'], {})
        step = int(row['step'])
        if step in path:
            raise ValueError('duplicate planned step')
        path[step] = point
    reports = []
    resolved_paths = {}
    for plan_id, variants in groups.items():
        paths = {}
        for variant in ['independent', 'resolved']:
            if set(variants.get(variant, {})) != set(robot_ids):
                raise ValueError('plan does not cover every robot')
            paths[variant] = {}
            for robot in robot_ids:
                steps = variants[variant][robot]
                if sorted(steps) != list(range(len(steps))):
                    raise ValueError('non-contiguous plan time indices')
                path = [steps[index] for index in range(len(steps))]
                if any(static_map.clearance(point) < 0.399 for point in path):
                    raise ValueError('planned vertex intersects inflated static map')
                for start, end in zip(path, path[1:]):
                    distance = math.dist(start, end)
                    diagonal = (abs(start[0] - end[0]) > 1e-6 and
                                abs(start[1] - end[1]) > 1e-6)
                    if (distance > 1e-8 and abs(distance - 1.0) > 1e-6) or diagonal:
                        raise ValueError('plan step is not a grid edge or wait')
                    for fraction in np.linspace(0, 1, max(2, int(distance / 0.025) + 1)):
                        point = [start[i] + fraction * (end[i] - start[i]) for i in range(2)]
                        if static_map.clearance(point) < 0.399:
                            raise ValueError('planned segment intersects inflated static map')
                paths[variant][robot] = path
        for robot in robot_ids:
            independent, resolved = paths['independent'][robot], paths['resolved'][robot]
            if independent[0] != resolved[0] or independent[-1] != resolved[-1]:
                raise ValueError('resolved plan changed a robot start or goal')
        initial = count_path_conflicts(list(paths['independent'].values()))
        final = count_path_conflicts(list(paths['resolved'].values()))
        if final:
            raise ValueError('unresolved swept-volume conflicts in joint plan')
        reports.append({'plan_id': plan_id, 'initial_conflicts': initial,
                        'final_conflicts': final})
        resolved_paths[plan_id] = paths['resolved']
    if not reports:
        raise ValueError('no joint plans recorded')
    return reports, resolved_paths


def audit_jobs(runtime, scenario, samples, completed):
    jobs = {job['id']: job for job in scenario['jobs']}
    stations = {station['id']: station for station in scenario['stations']}
    if len(jobs) != len(scenario['jobs']) or len(stations) != len(scenario['stations']):
        raise ValueError('duplicate scenario job or station identifiers')
    states, owners, services, checks = {}, {}, {}, []
    events = read_rows(Path(runtime) / 'events.csv')
    previous_time = -math.inf
    for event in events:
        timestamp = float(event['sim_time'])
        if not math.isfinite(timestamp) or timestamp < previous_time or timestamp < 0:
            raise ValueError('invalid or nonmonotonic event timestamp')
        previous_time = timestamp
    start = next((float(event['sim_time']) for event in events
                  if event['event'] == 'fleet_started'), None)
    if start is None:
        raise ValueError('missing fleet_started event')
    for trajectory in samples.values():
        if any(len(sample) < 6 or not all(map(math.isfinite, sample)) for sample in trajectory):
            raise ValueError('invalid physical service measurements')
        if any(after[0] <= before[0] for before, after in zip(trajectory, trajectory[1:])):
            raise ValueError('nonmonotonic physical service measurements')
    for event in events:
        action = event['event']
        if action not in ['assigned', 'pickup_service', 'dropoff_service', 'picked', 'delivered']:
            continue
        job, robot = event['job_id'], event['robot_id']
        time = float(event['sim_time'])
        if job not in jobs or robot not in samples:
            raise ValueError('unknown job or robot in mission ledger')
        if action == 'assigned':
            if job in states or robot in owners.values():
                raise ValueError('duplicate job ownership or overloaded robot')
            if time - start + 0.05 < jobs[job].get('release_time', 0):
                raise ValueError('job assigned before release')
            states[job], owners[job] = 'assigned', robot
            continue
        pickup = action in ['pickup_service', 'picked']
        expected = 'assigned' if pickup else 'picked'
        if states.get(job) != expected or owners.get(job) != robot:
            raise ValueError('invalid pickup/delivery/ownership sequence')
        service_name = 'pickup_service' if pickup else 'dropoff_service'
        if action == service_name:
            if job in services:
                raise ValueError('cargo service started twice without completion')
            services[job] = (service_name, time)
            continue
        if job not in services or services[job][0] != service_name:
            raise ValueError('missing matching cargo service start')
        service_start = services.pop(job)[1]
        required = float(jobs[job].get(service_name, 1.0))
        if not math.isfinite(required) or required < 0:
            raise ValueError('invalid configured cargo service duration')
        # События ставятся в разных callbacks; один физический шаг равен 20 мс.
        if time - service_start + 0.03 < required:
            raise ValueError('configured cargo service duration was not observed')
        states[job] = action
        if action == 'delivered':
            del owners[job]
        nearest = min(samples[robot], key=lambda sample: abs(sample[0] - time), default=None)
        if nearest is None or abs(nearest[0] - time) > 0.15:
            raise ValueError('no physical position evidence at service time')
        station = stations[jobs[job]['pickup' if pickup else 'dropoff']]
        error = math.dist(nearest[1:3], (station['x'], station['y']))
        if error > 0.20:
            raise ValueError('physical service position is outside station tolerance')
        window = [sample for sample in samples[robot] if service_start <= sample[0] <= time]
        if not window and time - service_start <= 0.15:
            window = [nearest]
        if (not window or abs(window[0][0] - service_start) > 0.15 + 1e-8 or
                abs(time - window[-1][0]) > 0.15 + 1e-8):
            raise ValueError('missing physical coverage of cargo service window')
        for sample in window:
            if math.dist(sample[1:3], (station['x'], station['y'])) > 0.20:
                raise ValueError('robot left station during cargo service')
            if abs(sample[4]) >= 0.05 or abs(sample[5]) >= 0.10:
                raise ValueError('robot was physically moving during cargo service')
        for before, after in zip(window, window[1:]):
            gap = after[0] - before[0]
            if gap > 0.15 + 1e-8:
                raise ValueError('missing physical samples inside cargo service window')
            if math.dist(before[1:3], after[1:3]) / gap > 0.05:
                raise ValueError('physical displacement during cargo service')
        checks.append({'job_id': job, 'robot_id': robot, 'event': action,
                       'sim_time': time, 'position_error_m': error,
                       'service_duration_s': time - service_start,
                       'service_samples': len(window)})
    all_delivered = (set(states) == set(jobs) and
                     all(state == 'delivered' for state in states.values()))
    if completed and not all_delivered:
        raise ValueError('not all jobs were delivered exactly once')
    return checks, events


def audit_rounds(events, paths, samples, allow_interrupted=False):
    active = None
    intervals = []
    last_steps = {}
    for trajectory in samples.values():
        if any(len(sample) < 6 or not all(map(math.isfinite, sample))
               for sample in trajectory) or any(
                after[0] <= before[0] for before, after in zip(trajectory, trajectory[1:])):
            raise ValueError('invalid physical sample times')
    last_event_time = -math.inf
    for event in events:
        if event['event'] not in ['round_started', 'round_completed']:
            continue
        values = dict(part.split('=', 1) for part in event['detail'].split(';') if '=' in part)
        key = (values['plan'], int(values['step']))
        time = float(event['sim_time'])
        if not math.isfinite(time) or time < last_event_time:
            raise ValueError('invalid round time')
        last_event_time = time
        if event['event'] == 'round_started':
            if active is not None:
                raise ValueError('overlapping joint execution rounds')
            if key[1] != last_steps.get(key[0], 0) + 1:
                raise ValueError('replayed or skipped execution round')
            active = (key, time)
        elif active is None or active[0] != key:
            raise ValueError('round completion without matching start')
        else:
            if time <= active[1]:
                raise ValueError('nonpositive execution round duration')
            intervals.append((key, active[1], time, True))
            last_steps[key[0]] = key[1]
            active = None
    if active is not None:
        if not allow_interrupted:
            raise ValueError('unfinished execution round')
        finish = min(trajectory[-1][0] for trajectory in samples.values() if trajectory)
        intervals.append((active[0], active[1], finish, False))
    if not allow_interrupted:
        for plan_id, robot_paths in paths.items():
            expected_steps = max(len(path) for path in robot_paths.values()) - 1
            if last_steps.get(plan_id, 0) != expected_steps:
                raise ValueError('joint plan was not fully executed')
    maximum = 0.0
    maximum_gap = 0.0
    for (plan_id, step), start_time, end_time, completed in intervals:
        if plan_id not in paths:
            raise ValueError('executed round references unknown plan')
        for robot, path in paths[plan_id].items():
            if step < 1 or step >= len(path):
                raise ValueError('executed step is outside recorded path')
            trajectory = [sample for sample in samples[robot]
                          if start_time <= sample[0] <= end_time]
            if not trajectory:
                raise ValueError('missing physical evidence inside an execution round')
            if (trajectory[0][0] - start_time > 0.06 + 1e-8 or
                    end_time - trajectory[-1][0] > 0.06 + 1e-8):
                raise ValueError('missing physical samples at execution round endpoints')
            for before, after in zip(trajectory, trajectory[1:]):
                gap = after[0] - before[0]
                maximum_gap = max(maximum_gap, gap)
                if gap > 0.06 + 1e-8:
                    raise ValueError('missing physical samples inside an execution round')
            if math.dist(trajectory[0][1:3], path[step - 1]) > 0.12:
                raise ValueError('execution round did not start at its planned vertex')
            if completed:
                if math.dist(trajectory[-1][1:3], path[step]) > 0.12:
                    raise ValueError('execution round ended before reaching its planned vertex')
                stopped = [sample for sample in trajectory if sample[0] >= end_time - 0.2]
                if len(stopped) < 5 or stopped[-1][0] - stopped[0][0] < 0.15:
                    raise ValueError('missing physical stop evidence at execution barrier')
                for before, after in zip(stopped, stopped[1:]):
                    speed = math.dist(before[1:3], after[1:3]) / (after[0] - before[0])
                    reported_motion = (len(after) >= 6 and
                                       (after[4] >= 0.05 or abs(after[5]) >= 0.10))
                    if speed > 0.05 or reported_motion:
                        raise ValueError('robot still moving at execution barrier')
            for sample in trajectory:
                deviation = point_segment_distance(sample[1:3], path[step - 1], path[step])
                maximum = max(maximum, deviation)
                if deviation > 0.12:
                    raise ValueError('physical trajectory left the planned edge corridor')
    if not intervals:
        raise ValueError('no completed coordinated execution rounds')
    for trajectory in samples.values():
        for before, after in zip(trajectory, trajectory[1:]):
            if any(start - 0.02 <= after[0] <= end + 0.02
                   for _, start, end, _ in intervals):
                continue
            speed = math.dist(before[1:3], after[1:3]) / (after[0] - before[0])
            if speed > 0.05:
                raise ValueError('physical movement outside coordinated execution rounds')
    return {'verified_rounds': len(intervals), 'max_edge_deviation_m': maximum,
            'completed_rounds': sum(completed for _, _, _, completed in intervals),
            'interrupted_rounds': sum(not completed for _, _, _, completed in intervals),
            'max_physical_sample_gap_s': maximum_gap}


def audit_assignments(runtime):
    reports = []
    for path in sorted(Path(runtime).glob('assignment_*.yaml')):
        data = yaml.safe_load(path.read_text())
        robots, jobs, costs = data['robot_ids'], data['job_ids'], data['cost_matrix']
        if len(robots) > 6 or len(jobs) > 6:
            raise ValueError('independent assignment oracle supports at most six robots and jobs')
        if len(costs) != len(robots) or any(len(row) != len(jobs) for row in costs):
            raise ValueError('invalid assignment cost matrix dimensions')
        if any(not isinstance(cost, int) for row in costs for cost in row):
            raise ValueError('assignment costs must be integer grid distances')
        chosen = data['chosen_pairs']
        used_robots, used_jobs, total = set(), set(), 0
        for pair in chosen:
            robot, job = pair['robot_id'], pair['job_id']
            if robot not in robots or job not in jobs or robot in used_robots or job in used_jobs:
                raise ValueError('invalid or duplicate matching pair')
            cost = costs[robots.index(robot)][jobs.index(job)]
            if cost < 0 or pair['cost'] != cost:
                raise ValueError('chosen assignment uses unreachable or incorrect cost')
            total += cost
            used_robots.add(robot)
            used_jobs.add(job)
        if total != data['total_cost']:
            raise ValueError('incorrect matching total cost')
        best = (0, 0)

        def enumerate_matching(index, used, count, value):
            nonlocal best
            if index == len(robots):
                candidate = (-count, value)
                best = min(best, candidate)
                return
            enumerate_matching(index + 1, used, count, value)
            for job, cost in enumerate(costs[index]):
                if job not in used and cost >= 0:
                    enumerate_matching(index + 1, used | {job}, count + 1, value + cost)

        enumerate_matching(0, set(), 0, 0)
        if (-len(chosen), total) != best:
            raise ValueError('matching is not maximum-cardinality minimum-cost')
        reports.append({'file': path.name, 'assigned': len(chosen), 'optimal_cost': total})
    if not reports:
        raise ValueError('missing assignment evidence')
    return reports
