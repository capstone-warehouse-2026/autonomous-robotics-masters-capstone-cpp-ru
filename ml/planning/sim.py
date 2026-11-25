"""Synthetic warehouse episodes and delay labels for the G3 learned cost (method B).

No ROS, no Webots: numpy only, so labels are reproducible from a seed. An episode is

1. a warehouse layout (rack rows, aisles, pallets) on the 0.05 m grid of the planner, origin (-10, -10);
2. moving agents (people, forklifts): they travel between their stations along shortest paths and
   dwell there, so the places where agents gather are partly predictable from the recent past;
3. an observer robot at time t0 whose local perception window (10 x 10 m, as G2) records the
   occupancy history at t0, t0 - 1 s, ... -- the only dynamic input of the network;
4. probe robots driven by a fixed classical controller (follow the shortest path at 0.5 m/s, stop
   while an agent is ahead within the stopping distance) that start at t0 and run into the future.
   The extra time they spend in every coarse cell, per metre travelled, is the delay label.

The network never sees the future agent trajectories, only the history (projects/03_planning.md).
Biases of these labels are listed in ml/planning/README.md and in the data card.
"""

import heapq
import math

import numpy as np

RESOLUTION = 0.05      # m, fine grid of the planner (/map)
BLOCK = 4              # fine cells per coarse cell side: the network runs on 0.2 m cells
MAP_SIZE = 20.0        # m
ORIGIN = (-10.0, -10.0)
HISTORY = 4            # frames: t0, t0 - 1 s, t0 - 2 s, t0 - 3 s
HISTORY_PERIOD = 1.0   # s
WINDOW = 10.0          # m, local perception window centred on the observer (G2 grid size)
DT = 0.1               # s, simulation step
ROBOT_RADIUS = 0.272   # m, docs/adr/0002
AGENT_RADIUS = 0.3     # m
ROBOT_SPEED = 0.5      # m/s, nominal speed of the probe controller (G4 limit)
STOP_DISTANCE = 1.0    # m between centres: an agent ahead closer than this stops the probe
CONTACT_DISTANCE = ROBOT_RADIUS + AGENT_RADIUS + 0.1  # m: stop whatever the direction
PROBE_TIMEOUT = 90.0   # s
MAX_LABEL = 10.0       # extra cost per metre (= extra seconds per metre * nominal speed)

# Channels of the network input, in this order (C++: capstone_planning::LearnedCost).
CHANNELS = ['static'] + [f'observed_t-{k}' for k in range(HISTORY)] + ['observed_mask']


# --- grids --------------------------------------------------------------------------------------

def cells(size_m=MAP_SIZE):
    return int(round(size_m / RESOLUTION))


def fill_rect(grid, x0, y0, x1, y1, value=1):
    """Sets cells whose centre lies in [x0, x1] x [y0, y1] (map frame, m)."""
    n_y, n_x = grid.shape
    xs = ORIGIN[0] + (np.arange(n_x) + 0.5) * RESOLUTION
    ys = ORIGIN[1] + (np.arange(n_y) + 0.5) * RESOLUTION
    ix = np.nonzero((xs >= x0) & (xs <= x1))[0]
    iy = np.nonzero((ys >= y0) & (ys <= y1))[0]
    if ix.size and iy.size:
        grid[iy[0]:iy[-1] + 1, ix[0]:ix[-1] + 1] = value


def pool(fine, block=BLOCK):
    """Mean over block x block cells; partial blocks at the border average the existing cells.
    The same definition as capstone_planning::pool_fraction in C++."""
    n_y, n_x = fine.shape
    c_y, c_x = -(-n_y // block), -(-n_x // block)
    padded = np.zeros((c_y * block, c_x * block), np.float64)
    count = np.zeros_like(padded)
    padded[:n_y, :n_x] = fine
    count[:n_y, :n_x] = 1.0
    s = padded.reshape(c_y, block, c_x, block).sum(axis=(1, 3))
    c = count.reshape(c_y, block, c_x, block).sum(axis=(1, 3))
    return (s / c).astype(np.float32)


def disc_mask(shape, centres, radius):
    """Fine cells whose centre is within `radius` of any centre (map frame)."""
    mask = np.zeros(shape, bool)
    r_cells = int(math.ceil(radius / RESOLUTION)) + 1
    for x, y in centres:
        cx = int((x - ORIGIN[0]) / RESOLUTION)
        cy = int((y - ORIGIN[1]) / RESOLUTION)
        x0, x1 = max(cx - r_cells, 0), min(cx + r_cells + 1, shape[1])
        y0, y1 = max(cy - r_cells, 0), min(cy + r_cells + 1, shape[0])
        if x0 >= x1 or y0 >= y1:
            continue
        gx = ORIGIN[0] + (np.arange(x0, x1) + 0.5) * RESOLUTION
        gy = ORIGIN[1] + (np.arange(y0, y1) + 0.5) * RESOLUTION
        d2 = (gx[None, :] - x) ** 2 + (gy[:, None] - y) ** 2
        mask[y0:y1, x0:x1] |= d2 <= radius * radius
    return mask


# --- layout ---------------------------------------------------------------------------------------

def generate_layout(rng):
    """Random warehouse: border walls, 3-5 rack rows along x or y with 1.6-2.6 m aisles, a cross
    aisle through every row, a few pallets. Returns the fine occupancy grid (uint8, 1 = obstacle)."""
    n = cells()
    grid = np.zeros((n, n), np.uint8)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 1
    rows = int(rng.integers(3, 6))
    depth = rng.uniform(0.6, 1.0)
    aisle = rng.uniform(1.6, 2.6)
    pitch = depth + aisle
    first = -0.5 * (rows - 1) * pitch + rng.uniform(-1.0, 1.0)
    start, end = rng.uniform(-7.5, -5.0), rng.uniform(5.0, 7.5)
    cross = rng.uniform(-3.0, 3.0)
    cross_half = 0.5 * rng.uniform(1.4, 2.4)
    along_x = rng.random() < 0.5
    for r in range(rows):
        c = first + r * pitch
        for a0, a1 in ((start, cross - cross_half), (cross + cross_half, end)):
            if along_x:
                fill_rect(grid, a0, c - 0.5 * depth, a1, c + 0.5 * depth)
            else:
                fill_rect(grid, c - 0.5 * depth, a0, c + 0.5 * depth, a1)
    for _ in range(int(rng.integers(2, 6))):
        x, y = rng.uniform(-8.5, 8.5, 2)
        w, h = rng.uniform(0.6, 1.4, 2)
        fill_rect(grid, x, y, x + w, y + h)
    return grid


# --- shortest paths on the coarse grid -------------------------------------------------------------

def inflate(grid, radius):
    """Fine cells closer than `radius` to an obstacle cell (brute force over a disc kernel)."""
    r = int(math.ceil(radius / RESOLUTION))
    out = grid.astype(bool).copy()
    ys, xs = np.nonzero(grid)
    if ys.size == 0:
        return out
    offsets = [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
               if (dy * dy + dx * dx) * RESOLUTION ** 2 <= radius * radius]
    n_y, n_x = grid.shape
    for dy, dx in offsets:
        yy = np.clip(ys + dy, 0, n_y - 1)
        xx = np.clip(xs + dx, 0, n_x - 1)
        out[yy, xx] = True
    return out


class CoarseGraph:
    """8-connected graph on 0.2 m cells that are entirely free after inflation by `radius`."""

    def __init__(self, grid, radius):
        blocked = pool(inflate(grid, radius).astype(np.float32)) > 0.0
        self.free = ~blocked
        self.shape = self.free.shape
        self.size = RESOLUTION * BLOCK

    def centre(self, cell):
        return (ORIGIN[0] + (cell[1] + 0.5) * self.size, ORIGIN[1] + (cell[0] + 0.5) * self.size)

    def cell(self, point):
        return (int((point[1] - ORIGIN[1]) / self.size), int((point[0] - ORIGIN[0]) / self.size))

    def random_free(self, rng):
        ys, xs = np.nonzero(self.free)
        i = int(rng.integers(ys.size))
        return int(ys[i]), int(xs[i])

    def shortest_path(self, a, b):
        """Dijkstra with octile steps, no corner cutting. Returns cells a..b or None."""
        if not (self.free[a] and self.free[b]):
            return None
        n_y, n_x = self.shape
        dist = {a: 0.0}
        parent = {a: None}
        heap = [(0.0, a)]
        while heap:
            d, c = heapq.heappop(heap)
            if c == b:
                break
            if d > dist[c]:
                continue
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    n = (c[0] + dy, c[1] + dx)
                    if not (0 <= n[0] < n_y and 0 <= n[1] < n_x and self.free[n]):
                        continue
                    if dy and dx and not (self.free[c[0] + dy, c[1]] and self.free[c[0], c[1] + dx]):
                        continue
                    nd = d + (math.sqrt(2.0) if dy and dx else 1.0)
                    if nd < dist.get(n, math.inf):
                        dist[n] = nd
                        parent[n] = c
                        heapq.heappush(heap, (nd, n))
        if b not in parent:
            return None
        path = [b]
        while parent[path[-1]] is not None:
            path.append(parent[path[-1]])
        return path[::-1]

    def polyline(self, path, step):
        """Centres of the path cells resampled every `step` m: array (N, 2)."""
        pts = np.array([self.centre(c) for c in path], np.float64)
        if len(pts) == 1:
            return pts
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        t = np.arange(0.0, s[-1] + 1e-9, step)
        return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], axis=1)


# --- agents -----------------------------------------------------------------------------------------

def simulate_agents(graph, rng, count, duration):
    """Positions (T, A, 2) every DT. Each agent has 2-3 stations (shared hot spots are likely),
    walks between them at its own speed and dwells 3-25 s at each."""
    steps = int(round(duration / DT)) + 1
    hot_spots = [graph.random_free(rng) for _ in range(int(rng.integers(2, 5)))]
    positions = np.zeros((steps, count, 2))
    for a in range(count):
        stations = [hot_spots[int(rng.integers(len(hot_spots)))] if rng.random() < 0.6 else graph.random_free(rng)
                    for _ in range(int(rng.integers(2, 4)))]
        speed = rng.uniform(0.4, 1.2)
        here = graph.random_free(rng)
        track = [np.array(graph.centre(here))]
        while len(track) < steps:
            target = stations[int(rng.integers(len(stations)))]
            path = graph.shortest_path(here, target)
            if path is None:
                target = graph.random_free(rng)
                path = graph.shortest_path(here, target) or [here]
            track.extend(graph.polyline(path, speed * DT)[1:])
            dwell = int(rng.uniform(3.0, 25.0) / DT)
            track.extend([np.array(graph.centre(target))] * dwell)
            here = target
        positions[:, a, :] = np.array(track[:steps])
    return positions


# --- probes (fixed classical controller) -----------------------------------------------------------

def run_probes(graph, paths, agents, t0_step):
    """Drives every probe along its polyline (sampled every ROBOT_SPEED * DT) from t0.

    Controller: move one sample per step; stop while an agent is closer than STOP_DISTANCE and
    ahead (angle < 90 deg to the direction of motion), or closer than CONTACT_DISTANCE anywhere.
    Returns per-probe arrays of (x, y, waited) for every step until arrival or timeout."""
    m = len(paths)
    longest = max(len(p) for p in paths)
    pts = np.zeros((m, longest + 1, 2))
    lengths = np.array([len(p) for p in paths])
    for i, p in enumerate(paths):
        pts[i, :len(p)] = p
        pts[i, len(p):] = p[-1]
    index = np.zeros(m, int)
    done = lengths <= 1
    log = [[] for _ in range(m)]
    max_steps = int(PROBE_TIMEOUT / DT)
    for k in range(max_steps):
        if done.all():
            break
        step = min(t0_step + k, agents.shape[0] - 1)
        pos = pts[np.arange(m), index]
        nxt = pts[np.arange(m), np.minimum(index + 1, lengths - 1)]
        heading = nxt - pos
        rel = agents[step][None, :, :] - pos[:, None, :]          # (M, A, 2)
        dist = np.linalg.norm(rel, axis=2)
        ahead = (rel * heading[:, None, :]).sum(axis=2) > 0.0
        blocked = ((dist < STOP_DISTANCE) & ahead).any(axis=1) | (dist < CONTACT_DISTANCE).any(axis=1)
        active = ~done
        for i in np.nonzero(active)[0]:
            log[i].append((pos[i, 0], pos[i, 1], bool(blocked[i])))
        move = active & ~blocked
        index[move] += 1
        done |= index >= lengths - 1
    return log, done


def delay_labels(graph_shape, logs, min_length=0.4):
    """Extra cost per metre in every coarse cell: (time spent - nominal time) * speed / length,
    i.e. waiting seconds converted to metres of nominal travel per metre. NaN where probes travelled
    less than `min_length` m in the cell (no supervision)."""
    wait = np.zeros(graph_shape)
    length = np.zeros(graph_shape)
    size = RESOLUTION * BLOCK
    for log in logs:
        for x, y, waited in log:
            r = int((y - ORIGIN[1]) / size)
            c = int((x - ORIGIN[0]) / size)
            if not (0 <= r < graph_shape[0] and 0 <= c < graph_shape[1]):
                continue
            if waited:
                wait[r, c] += DT
            else:
                length[r, c] += ROBOT_SPEED * DT
    label = np.full(graph_shape, np.nan, np.float32)
    ok = length >= min_length
    label[ok] = np.minimum(wait[ok] * ROBOT_SPEED / length[ok], MAX_LABEL)
    return label, length.astype(np.float32)


# --- network input ---------------------------------------------------------------------------------

def observation(static, agents, t0_step, observer):
    """Network input (len(CHANNELS), H, W) float32 on coarse cells, from the fine static grid, the
    agents up to t0 and the observer position (window centre)."""
    shape = static.shape
    window = np.zeros(shape, bool)
    half = 0.5 * WINDOW
    xs = ORIGIN[0] + (np.arange(shape[1]) + 0.5) * RESOLUTION
    ys = ORIGIN[1] + (np.arange(shape[0]) + 0.5) * RESOLUTION
    window[np.ix_((np.abs(ys - observer[1]) <= half), (np.abs(xs - observer[0]) <= half))] = True
    frames = []
    for k in range(HISTORY):
        step = max(t0_step - int(round(k * HISTORY_PERIOD / DT)), 0)
        occupied = disc_mask(shape, agents[step], AGENT_RADIUS) & window & (static == 0)
        frames.append(pool(occupied.astype(np.float32)))
    return np.stack([pool(static.astype(np.float32))] + frames + [pool(window.astype(np.float32))])


def episode(seed, layout_seed, probes=160, agents_range=(6, 15), t0=20.0):
    """One sample: dict with `input` (C, H, W), `label` (H, W, NaN = unsupervised), `length` (H, W),
    the fine `static` grid, agent trajectories and probe statistics."""
    rng = np.random.default_rng(seed)
    static = generate_layout(np.random.default_rng(layout_seed))
    agent_graph = CoarseGraph(static, AGENT_RADIUS)
    robot_graph = CoarseGraph(static, ROBOT_RADIUS + 0.05)
    t0_step = int(round(t0 / DT))
    agents = simulate_agents(agent_graph, rng, int(rng.integers(*agents_range)), t0 + PROBE_TIMEOUT + 5.0)
    observer = robot_graph.centre(robot_graph.random_free(rng))
    paths = []
    while len(paths) < probes:
        a, b = robot_graph.random_free(rng), robot_graph.random_free(rng)
        path = robot_graph.shortest_path(a, b)
        if path is not None and len(path) > 5:
            paths.append(robot_graph.polyline(path, ROBOT_SPEED * DT))
    logs, arrived = run_probes(robot_graph, paths, agents, t0_step)
    label, length = delay_labels(robot_graph.shape, logs)
    return {
        'input': observation(static, agents, t0_step, observer),
        'label': label,
        'length': length,
        'static': static,
        'agents': agents.astype(np.float32),
        't0_step': t0_step,
        'observer': np.array(observer, np.float32),
        'arrived_fraction': float(arrived.mean()),
    }
