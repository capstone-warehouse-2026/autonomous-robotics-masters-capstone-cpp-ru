"""Records RGB-D frames for method B from the running simulator (offline data preparation).

Runs inside the simulator container as the extern Webots supervisor "dataset_recorder" of a world
prepared by run_recording.sh, and at the same time as a ROS node that reads /sensors/rgb,
/sensors/depth and /sensors/camera_info: the frames are exactly what CnnClassifier receives at
runtime (rgb8, depth registered to the colour camera, NaN = no measurement).

For every sample the supervisor teleports the robot to a random pose in the arena, sets a random
light level, lets physics settle and rejects poses where the robot was pushed by an obstacle. The
exact robot pose (ground truth) is stored only for labelling and analysis, never as a network input.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from controller import Supervisor
from message_filters import Subscriber, TimeSynchronizer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from labels import pose_matrix, quaternion_pose

ROBOT_Z = 0.095  # TiagoBase origin above the floor (simulation/worlds/warehouse.wbt)


class FrameCollector(Node):
    """Keeps the latest synchronized RGB-D frame and reads the static camera TF."""

    def __init__(self):
        super().__init__('g2_dataset_recorder', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self._lock = threading.Lock()
        self._latest = None
        subscribers = [
            Subscriber(self, Image, '/sensors/rgb', qos_profile=qos_profile_sensor_data),
            Subscriber(self, Image, '/sensors/depth', qos_profile=qos_profile_sensor_data),
            Subscriber(self, CameraInfo, '/sensors/camera_info', qos_profile=qos_profile_sensor_data),
        ]
        self._sync = TimeSynchronizer(subscribers, 10)
        self._sync.registerCallback(self._on_frame)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_frame(self, rgb, depth, info):
        stamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9
        with self._lock:
            self._latest = (stamp, rgb, depth, info)

    def latest(self):
        with self._lock:
            return self._latest

    def base_from_camera(self, camera_frame):
        transform = self.tf_buffer.lookup_transform('base_link', camera_frame, Time()).transform
        t, q = transform.translation, transform.rotation
        return quaternion_pose([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def to_arrays(rgb, depth):
    image = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
    depth_m = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    return image.copy(), depth_m.copy()


class Recorder:
    def __init__(self, args):
        self.args = args
        self.supervisor = Supervisor()
        self.step_ms = int(self.supervisor.getBasicTimeStep())
        self.robot = self.supervisor.getFromDef('DATASET_ROBOT')
        self.translation = self.robot.getField('translation')
        self.rotation = self.robot.getField('rotation')
        self.lights = [self.supervisor.getFromDef(name).getField('luminosity')
                       for name in ('DATASET_BACKGROUND', 'DATASET_LIGHT')]
        floor_size = self.supervisor.getFromDef('DATASET_ARENA').getField('floorSize').getSFVec2f()
        self.half_x = floor_size[0] / 2 - args.wall_margin
        self.half_y = floor_size[1] / 2 - args.wall_margin
        self.rng = np.random.default_rng(args.seed)

        rclpy.init()
        self.collector = FrameCollector()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.collector)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def step(self, seconds):
        for _ in range(max(1, round(seconds * 1000 / self.step_ms))):
            if self.supervisor.step(self.step_ms) == -1:
                raise SystemExit('Webots stopped')

    def wait_frame(self, after, timeout):
        """Steps the simulation until a frame taken at or after `after` (sim time) arrives."""
        deadline = self.supervisor.getTime() + timeout
        while self.supervisor.getTime() < deadline:
            frame = self.collector.latest()
            if frame is not None and frame[0] >= after - 1e-6:
                return frame
            self.step(0.02)
            time.sleep(0.001)  # let the ROS thread deliver messages
        return None

    def place(self, x, y, yaw, light):
        # Restore the whole robot first, joints included: a teleport into an obstacle can leave the wheels
        # and casters displaced, and the robot would no longer stand at ROBOT_Z on later poses.
        self.robot.loadState('initial')
        self.translation.setSFVec3f([x, y, ROBOT_Z])
        self.rotation.setSFRotation([0.0, 0.0, 1.0, yaw])
        self.robot.resetPhysics()
        for field in self.lights:
            field.setSFFloat(light)

    def run(self):
        args = self.args
        os.makedirs(args.out, exist_ok=True)
        first = self.wait_frame(0.0, timeout=60.0)
        if first is None:
            raise SystemExit('no frames on /sensors/* within 60 s of simulation time')
        camera_frame = first[2].header.frame_id
        self.robot.saveState('initial')
        info = first[3]
        base_from_camera = self.collector.base_from_camera(camera_frame)

        saved = blocked = no_frame = missed_in_row = 0
        while saved < args.samples and blocked + no_frame < args.samples * 20:
            x = self.rng.uniform(-self.half_x, self.half_x)
            y = self.rng.uniform(-self.half_y, self.half_y)
            yaw = self.rng.uniform(-math.pi, math.pi)
            light = self.rng.uniform(args.light_min, args.light_max)
            self.place(x, y, yaw, light)
            self.step(args.settle)
            position = self.robot.getPosition()
            if math.hypot(position[0] - x, position[1] - y) > 0.02 or abs(position[2] - ROBOT_Z) > 0.02:
                blocked += 1  # pushed by an obstacle: the pose was not free
                if blocked % 500 == 0:
                    print(f'{blocked} poses blocked; last moved to {[round(v, 3) for v in position]} '
                          f'from {[round(x, 3), round(y, 3), ROBOT_Z]}', flush=True)
                continue
            frame = self.wait_frame(self.supervisor.getTime(), timeout=1.0)
            if frame is None:
                no_frame += 1
                missed_in_row += 1
                if missed_in_row >= 20:
                    print(f'FAIL: no camera frames for 20 poses in a row after {saved} frames '
                          f'(see simulator.log)', flush=True)
                    break
                continue
            missed_in_row = 0
            stamp, rgb, depth, _ = frame
            image, depth_m = to_arrays(rgb, depth)
            world_from_base = pose_matrix(self.robot.getPosition(), self.robot.getOrientation())
            np.savez_compressed(
                os.path.join(args.out, f'frame_{saved:06d}.npz'),
                rgb=image, depth=depth_m, world_from_base=world_from_base,
                stamp=stamp, light=light, sample_seed=args.seed, sample_index=saved)
            saved += 1
            if saved % 50 == 0:
                print(f'{saved}/{args.samples} frames, {blocked} poses blocked, {no_frame} without frame', flush=True)

        meta = {
            'world': args.world,
            'layout_id': args.layout_id,
            'seed': args.seed,
            'samples': saved,
            'rejected_poses': blocked + no_frame,
            'blocked_poses': blocked,
            'poses_without_frame': no_frame,
            'light_range': [args.light_min, args.light_max],
            'settle_s': args.settle,
            'camera_frame': camera_frame,
            'camera': {'fx': info.k[0], 'fy': info.k[4], 'cx': info.k[2], 'cy': info.k[5],
                       'width': info.width, 'height': info.height},
            'base_from_camera': base_from_camera.tolist(),
            'git_commit': os.environ.get('GIT_COMMIT', 'unknown'),
            'git_dirty': os.environ.get('GIT_DIRTY', 'unknown'),
            'recorded_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        }
        with open(os.path.join(args.out, 'meta.json'), 'w') as stream:
            json.dump(meta, stream, indent=2)
        complete = saved == args.samples
        print(f'{"done" if complete else "FAIL"}: {saved}/{args.samples} frames, {blocked} poses blocked, '
              f'{no_frame} without frame -> {args.out}', flush=True)
        # Everything is written. Leave at once: simulationQuit() or the normal teardown of the Webots
        # controller library and the ROS thread end the process with an error although the dataset is
        # complete. The simulator itself is stopped by run_recording.sh.
        sys.stdout.flush()
        os._exit(0 if complete else 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--out', required=True, help='output directory')
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--world', default='warehouse.wbt')
    parser.add_argument('--layout-id', default='warehouse')
    parser.add_argument('--light-min', type=float, default=0.2, help='lowest luminosity (low light)')
    parser.add_argument('--light-max', type=float, default=1.0)
    parser.add_argument('--settle', type=float, default=0.3, help='s of simulation after a teleport')
    parser.add_argument('--wall-margin', type=float, default=0.5, help='m kept free near the walls')
    Recorder(parser.parse_args()).run()


if __name__ == '__main__':
    main()
