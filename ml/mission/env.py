"""Дешёвая событийная среда: выдача заявок и поездки по фиксированным таблицам."""

import copy

import numpy as np

from spec import observation, WAIT_ACTION

FAMILIES = ('low', 'high', 'bottleneck', 'blocked', 'unequal')
SPLIT_SEEDS = {'train': (0, 999), 'validation': (1000, 1099), 'test': (2000, 2019)}


def make_scenario(split, seed, family):
    """Независимые layout/таблицы для train, validation и test, без разбиения кадров."""
    if split not in SPLIT_SEEDS or family not in FAMILIES:
        raise ValueError('unknown split or scenario family')
    lower, upper = SPLIT_SEEDS[split]
    if not lower <= seed <= upper:
        raise ValueError(f'seed {seed} does not belong to {split}')
    layout_count = {'train': 3, 'validation': 1, 'test': 2}[split]
    layout_index = seed % layout_count
    layout_seed = {'train': 10000, 'validation': 20000, 'test': 30000}[split]
    layout_rng = np.random.default_rng(layout_seed + layout_index)
    rng = np.random.default_rng(seed * 11 + FAMILIES.index(family))
    coordinates = layout_rng.uniform(-7.0, 7.0, size=(6, 2))
    distance = np.linalg.norm(coordinates[:, None] - coordinates[None, :], axis=2)
    # Объезды фиксируются в layout, динамическая загруженность — в сценарии.
    detour = layout_rng.uniform(1.0, 1.6, size=(6, 6))
    distances = distance * (detour + detour.T) / 2.0
    congestion = np.ones((6, 6))
    if family in ('high', 'bottleneck', 'blocked'):
        congestion = rng.uniform(1.0, 1.6, size=(6, 6))
    if family == 'bottleneck':
        congestion[:3, 3:] = 2.5
        congestion[3:, :3] = 2.5
    blocked = np.zeros((6, 6), dtype=bool)
    if family == 'blocked':
        first, second = rng.choice(6, size=2, replace=False)
        blocked[first, second] = True
        blocked[second, first] = True
    count = 5 if family == 'low' else 12
    releases = np.sort(rng.uniform(0.0, 170.0, size=count))
    releases[0] = 0.0
    jobs = []
    for index, release in enumerate(releases):
        pickup, dropoff = rng.choice(6, size=2, replace=False)
        deadline_window = rng.uniform(35.0, 180.0)
        if family == 'unequal':
            deadline_window = 25.0 if index % 2 else 170.0
        jobs.append({
            'id': f'job_{index:03}', 'pickup': f's{pickup}', 'dropoff': f's{dropoff}',
            'release_time': float(release), 'due_time': float(release + deadline_window),
            'pickup_service': 2.0, 'dropoff_service': 2.0,
        })
    return {
        'schema_version': 1, 'episode_id': f'{split}_{family}_{seed}',
        'split': split, 'scenario_seed': seed, 'family': family,
        'layout_id': f'{split}_{layout_index}', 'horizon': 300.0, 'initial_station': 's0',
        'stations': [{'id': f's{i}', 'x': float(x), 'y': float(y)}
                     for i, (x, y) in enumerate(coordinates)],
        'travel_times': (distances / 0.5).tolist(), 'distances': distances.tolist(),
        'congestion': congestion.tolist(), 'blocked': blocked.tolist(), 'jobs': jobs,
    }


class DispatchEnvironment:
    """Активная доставка атомарна для политики: выбрать новую до её конца нельзя."""

    def __init__(self, scenario, congestion_feature=True):
        self.scenario = scenario
        self.congestion_feature = congestion_feature
        self.stations = {station['id']: index
                         for index, station in enumerate(scenario['stations'])}
        self.travel = np.asarray(scenario['travel_times'])
        self.distances = np.asarray(scenario['distances'])
        self.congestion = np.asarray(scenario['congestion'])
        self.blocked = np.asarray(scenario['blocked'])
        self.reset()

    def reset(self):
        self.now = 0.0
        self.position = self.stations[self.scenario['initial_station']]
        self.jobs = copy.deepcopy(self.scenario['jobs'])
        for job in self.jobs:
            job.update(state='scheduled', failures=0)
        self.delivered = 0
        self.on_time = 0
        self.failed = 0
        self.distance = 0.0
        self.idle = 0.0
        self.recoveries = 0
        self.navigation_failures = 0
        self.tardiness = []
        self.done = False
        self.release_jobs()
        return self.observe()

    def release_jobs(self):
        for job in self.jobs:
            if job['state'] == 'scheduled' and job['release_time'] <= self.now:
                job['state'] = 'released'

    def candidates(self):
        available = [job for job in self.jobs if job['state'] == 'released']
        return sorted(available, key=lambda job: (job['release_time'], job['id']))[:4]

    def estimates(self, job):
        pickup = self.stations[job['pickup']]
        dropoff = self.stations[job['dropoff']]
        pickup_time = self.travel[self.position, pickup] * self.congestion[self.position, pickup]
        delivery_time = self.travel[pickup, dropoff] * self.congestion[pickup, dropoff]
        return {
            **job, 'pickup_time': pickup_time, 'delivery_time': delivery_time,
            'service_time': job['pickup_service'] + job['dropoff_service'],
            'congestion': self.congestion[pickup, dropoff],
            'failures': self.navigation_failures,
        }

    def observe(self):
        station = self.scenario['stations'][self.position]
        candidates = [self.estimates(job) for job in self.candidates()]
        return observation(candidates, self.now, station['x'], station['y'],
                           congestion=self.congestion_feature)

    def classical_action(self):
        candidates = self.candidates()
        if not candidates:
            return WAIT_ACTION

        def priority(slot):
            job = candidates[slot]
            pickup = self.stations[job['pickup']]
            dropoff = self.stations[job['dropoff']]
            route = self.distances[self.position, pickup] + self.distances[pickup, dropoff]
            return job['due_time'], route, job['id']

        return min(range(len(candidates)), key=priority)

    def step(self, action):
        if self.done:
            raise ValueError('episode already terminated')
        candidates = self.candidates()
        if not 0 <= action <= WAIT_ACTION or (action != WAIT_ACTION and action >= len(candidates)):
            raise ValueError('masked action')
        start = self.now
        reward = -0.02
        if action == WAIT_ACTION:
            self.now = min(self.now + 1.0, self.scenario['horizon'])
            self.idle += self.now - start
            if candidates:
                reward -= 0.05
        else:
            reward += self.execute(candidates[action])
        self.release_jobs()
        self.done = (self.now >= self.scenario['horizon'] or
                     all(job['state'] in ('delivered', 'failed') for job in self.jobs))
        elapsed = self.now - start
        reward -= 0.01 * elapsed
        if self.done:
            reward -= 2.0 * sum(job['state'] in ('released', 'picked') for job in self.jobs)
        observation_value, mask = self.observe()
        return observation_value, mask, reward, self.done, elapsed

    def execute(self, job):
        pickup = self.stations[job['pickup']]
        dropoff = self.stations[job['dropoff']]
        # По две попытки на этап; после отказа с грузом эпизод прекращается.
        for target, service, state in ((pickup, job['pickup_service'], 'picked'),
                                       (dropoff, job['dropoff_service'], 'delivered')):
            if self.blocked[self.position, target]:
                self.now = min(self.now + 2.0, self.scenario['horizon'])
                job['failures'] += 2
                self.navigation_failures += 2
                job['state'] = 'failed'
                self.recoveries += 1
                self.failed += 1
                if state == 'delivered':
                    self.now = self.scenario['horizon']
                return -5.0
            travel_time = (self.travel[self.position, target]
                           * self.congestion[self.position, target])
            duration = travel_time + service
            remaining = self.scenario['horizon'] - self.now
            if duration > remaining:
                fraction = min(1.0, remaining / max(travel_time, 1e-9))
                self.distance += self.distances[self.position, target] * fraction
                self.now = self.scenario['horizon']
                return -2.0
            self.distance += self.distances[self.position, target]
            self.now += duration
            self.position = target
            job['state'] = state
        self.delivered += 1
        late = max(0.0, self.now - job['due_time'])
        self.tardiness.append(late)
        self.on_time += int(late == 0.0)
        return 5.0 + (3.0 if late == 0.0 else -min(3.0, late * 0.03))

    def summary(self):
        """Только отладка обучения; итоговые метрики сдачи считает C++ benchmark."""
        released = sum(job['release_time'] <= self.now for job in self.jobs)
        return {'delivered': self.delivered, 'released': released, 'on_time': self.on_time,
                'failed': self.failed, 'elapsed': self.now, 'distance': self.distance,
                'idle': self.idle, 'recoveries': self.recoveries}
