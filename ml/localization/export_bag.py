#!/usr/bin/env python3
"""Export frozen-map sensor events and SEPARATE evaluation truth from ROS 2 bags."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


GT_TOPIC = '/evaluation/ground_truth'
GT_TYPE = 'nav_msgs/msg/Odometry'
PILOT_EVIDENCE = 'evidence=nominal_pilot_not_training_or_scored'


def marked_pilot(bag):
    here = bag.resolve()
    for parent in (here, here.parent):
        if parent.name.startswith('hw1-pilot-'):
            return True
        provenance = parent / 'provenance.txt'
        if provenance.is_file() and PILOT_EVIDENCE in provenance.read_text().splitlines():
            return True
    return False


def dataset_source(split, bags, synthetic):
    if any(marked_pilot(bag) for bag in bags) and split != 'pilot':
        raise ValueError('Known pilot inputs require --split pilot; not train/validation/test')
    if synthetic:
        return 'synthetic_fixture'
    return 'recorded_pilot' if split == 'pilot' else 'recorded_rosbag'


def inventory(bag):
    paths = [bag] if bag.is_file() else sorted(
        f for f in bag.iterdir() if f.is_file() and
        (f.name == 'metadata.yaml' or f.name.endswith(('.db3', '.mcap', '.db3.zstd', '.mcap.zstd'))))
    if not paths:
        raise ValueError(f'No rosbag storage files found in {bag}')
    hashes = {}
    for path in paths:
        h = hashlib.sha256()
        with path.open('rb') as src:
            for chunk in iter(lambda: src.read(1024*1024), b''):
                h.update(chunk)
        hashes[path.name] = h.hexdigest()
    return hashes


def read_bag(path, storage, needed, *, sidecar=False, exclude_embedded=False):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    meta = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if sidecar and meta != {GT_TOPIC: GT_TYPE}:
        raise ValueError('GT sidecar must contain only /evaluation/ground_truth (nav_msgs/msg/Odometry)')
    if exclude_embedded and GT_TOPIC in meta:
        raise ValueError('With --ground-truth-bag, input bag must not also contain ground truth')
    types = {name: get_message(typ) for name, typ in meta.items() if name in needed}
    messages = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic in types:
            messages.append((topic, deserialize_message(raw, types[topic])))
    return messages

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('output', type=Path)
    for key in ['world', 'trajectory', 'scenario']:
        p.add_argument('--'+key, required=True)
    p.add_argument('--world-file', type=Path, required=True)
    p.add_argument('--synthetic-fixture', action='store_true', help='Mark generated test bags; training/evaluation will reject them')
    p.add_argument('--split', choices=['train', 'validation', 'test', 'pilot'], required=True)
    p.add_argument('--ground-truth-bag', type=Path,
                   help='Optional separate evaluator-only rosbag (no publisher is started)')
    p.add_argument('--ground-truth-storage', help='Defaults to --storage')
    p.add_argument('--initial', nargs=3, type=float, required=True,
                   help='Known map-frame spawn x y yaw, NOT a post-hoc GT alignment')
    p.add_argument('--gt-to-map', nargs=3, type=float, required=True,
                   help='Predeclared rigid transform world/GT to SLAM map; never fit to estimates')
    p.add_argument('--storage', default='sqlite3')
    args = p.parse_args()
    bags = [args.bag, args.ground_truth_bag] if args.ground_truth_bag else [args.bag]
    source = dataset_source(args.split, bags, args.synthetic_fixture)
    if args.ground_truth_storage and not args.ground_truth_bag:
        p.error('--ground-truth-storage requires --ground-truth-bag')
    if args.ground_truth_bag and args.ground_truth_bag.resolve() == args.bag.resolve():
        p.error('Ground truth sidecar must be a separate bag')
    bag_hashes = inventory(args.bag)
    gt_hashes = inventory(args.ground_truth_bag) if args.ground_truth_bag else None
    needed = {'/sensors/wheel_odom', '/sensors/imu', '/sensors/scan', '/map', '/tf_static'}
    messages = read_bag(args.bag, args.storage, needed if args.ground_truth_bag else needed | {GT_TOPIC},
                        exclude_embedded=bool(args.ground_truth_bag))
    if args.ground_truth_bag:
        gt_messages = read_bag(args.ground_truth_bag, args.ground_truth_storage or args.storage,
                               {GT_TOPIC}, sidecar=True)
        gt_stamps = [stamp(m) for _, m in gt_messages]
        wheel = [stamp(m) for name, m in messages if name == '/sensors/wheel_odom']
        if len(gt_stamps) < 2 or any(a >= b for a, b in zip(gt_stamps, gt_stamps[1:])):
            raise ValueError('GT sidecar needs strictly increasing simulated timestamps')
        if not wheel or max(wheel[0], gt_stamps[0]) >= min(wheel[-1], gt_stamps[-1]):
            raise ValueError('GT sidecar and sensor bag need overlapping simulated timestamps')
        if any(m.header.frame_id != 'map' or m.child_frame_id != 'base_link' for _, m in gt_messages):
            raise ValueError('GT sidecar requires frames map -> base_link')
        messages += gt_messages  # Only evaluating ground_truth.csv, never exported to events.txt.
    maps = [m for t, m in messages if t == '/map']
    if not maps:
        raise ValueError('No /map: record a frozen SLAM map, not an invented occupancy map')
    grid = maps[0]
    if grid.header.frame_id != 'map':
        raise ValueError('Map must be in map frame')
    if any(m.info != grid.info or m.data != grid.data for m in maps):
        raise ValueError('Map changed: A/B needs one frozen map, not an online SLAM stream')
    transforms = {}
    for topic, msg in messages:
        if topic == '/tf_static':
            for tf in msg.transforms:
                if tf.header.frame_id == 'base_link':
                    transforms[tf.child_frame_id] = tf.transform
    # Preserve recorded order. Sorting by time would hide /clock rewinds.
    args.output.mkdir(parents=True, exist_ok=False)
    imu = None
    last_wheel = None
    truth = []
    counts = {'wheel': 0, 'scan': 0, 'ground_truth': 0}
    with (args.output/'events.txt').open('w') as out:
        for topic, msg in messages:
            if topic == '/sensors/imu':
                imu = msg
            elif topic == '/sensors/wheel_odom':
                t = stamp(msg)
                if last_wheel is not None and t < last_wheel:
                    raise ValueError('Record one monotonic trajectory per bag; split clock-reset sessions')
                last_wheel = t
                if msg.child_frame_id != 'base_link':
                    raise ValueError('wheel child frame must be base_link')
                valid = (imu is not None and imu.header.frame_id in ('imu_link', 'base_link')
                         and 0 <= t-stamp(imu) <= .1
                         and imu.angular_velocity_covariance[0] >= 0
                         and math.isfinite(imu.angular_velocity.z))
                rate = imu.angular_velocity.z if valid else 0
                out.write(f'W {t:.17g} {msg.twist.twist.linear.x:.17g} '
                          f'{msg.twist.twist.angular.z:.17g} {int(valid)} {rate:.17g}\n')
                counts['wheel'] += 1
            elif topic == '/sensors/scan':
                frame = msg.header.frame_id
                x = y = a = 0
                if frame != 'base_link':
                    if frame not in transforms:
                        raise ValueError(f'Missing direct static base_link -> {frame}')
                    tf = transforms[frame]
                    if abs(tf.rotation.x)+abs(tf.rotation.y) > 1e-6:
                        raise ValueError('Nonplanar LiDAR extrinsic unsupported')
                    x, y, a = tf.translation.x, tf.translation.y, yaw(tf.rotation)
                points = []
                for i, r in enumerate(msg.ranges):
                    if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                        angle = a+msg.angle_min+i*msg.angle_increment
                        points.extend((x+r*math.cos(angle), y+r*math.sin(angle)))
                out.write(f'S {stamp(msg):.17g} {len(points)//2} '+
                          ' '.join(f'{v:.17g}' for v in points)+'\n')
                counts['scan'] += 1
            elif topic == '/evaluation/ground_truth':
                if msg.header.frame_id != 'map':
                    raise ValueError('GT must already be in map frame')
                pose = msg.pose.pose
                dx, dy, da = args.gt_to_map
                c, s = math.cos(da), math.sin(da)
                truth.append((stamp(msg), dx+c*pose.position.x-s*pose.position.y,
                              dy+s*pose.position.x+c*pose.position.y,
                              math.atan2(math.sin(da+yaw(pose.orientation)), math.cos(da+yaw(pose.orientation)))))
    if not counts['wheel'] or not counts['scan'] or not truth:
        raise ValueError('Required wheel/scan/GT stream missing')
    counts['ground_truth'] = len(truth)
    with (args.output/'ground_truth.csv').open('w') as out:
        writer = csv.writer(out); writer.writerow(['stamp', 'x', 'y', 'yaw']); writer.writerows(truth)
    origin = grid.info.origin
    with (args.output/'map.txt').open('w') as out:
        out.write(f'{grid.info.width} {grid.info.height} {grid.info.resolution:.17g} '
                  f'{origin.position.x:.17g} {origin.position.y:.17g} {yaw(origin.orientation):.17g}\n')
        out.write(' '.join(map(str, grid.data))+'\n')
    hashes = {name: hashlib.sha256((args.output/name).read_bytes()).hexdigest()
              for name in ['map.txt', 'events.txt', 'ground_truth.csv']}
    manifest = dict(schema=1, source=source,
                    world_sha256=hashlib.sha256(args.world_file.read_bytes()).hexdigest(), bag=str(args.bag.resolve()),
                    world=args.world, trajectory=args.trajectory, scenario=args.scenario,
                    split=args.split, initial=args.initial, gt_to_map=args.gt_to_map,
                    counts=counts, sha256=hashes, bag_file_sha256=bag_hashes,
                    ground_truth_bag=str(args.ground_truth_bag.resolve()) if args.ground_truth_bag else None,
                    ground_truth_bag_file_sha256=gt_hashes,
                    eligible_for_scored_pipeline=(source == 'recorded_rosbag'))
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(counts))

if __name__ == '__main__':
    main()
