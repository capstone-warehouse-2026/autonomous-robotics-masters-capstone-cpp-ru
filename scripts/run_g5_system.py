#!/usr/bin/env python3
"""Поднять физический стенд, дождаться независимой проверки и убрать процессы."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]


class SystemRun:
    def __init__(self, args):
        self.args = args
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output / 'logs').mkdir()
        self.processes = []
        self.logs = []
        self.env = os.environ.copy()
        self.env.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                        ROS_LOG_DIR=str(self.output / 'logs' / 'ros'),
                        RCUTILS_COLORIZED_OUTPUT='0')
        self.webots = Path(self.env.get('WEBOTS_HOME', str(ROOT / '.cache' / 'webots')))
        self.env['WEBOTS_HOME'] = str(self.webots)
        self.env['PATH'] = str(ROOT / '.cache/native-deps/usr/bin') + ':' + self.env['PATH']

    def executable(self, package, name):
        path = ROOT / 'ros2_ws/install' / package / 'lib' / package / name
        if not path.is_file():
            raise RuntimeError(f'Нет {path}; выполните make g5-system-build')
        return str(path)

    def start(self, name, command, env=None, essential=True):
        stream = (self.output / 'logs' / f'{name}.log').open('w')
        self.logs.append(stream)
        process = subprocess.Popen([str(value) for value in command], env=env or self.env,
                                   cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes.append((name, process, essential))
        return process

    def node(self, name, package, executable, params, essential=True):
        command = [self.executable(package, executable), '--ros-args']
        for key, value in params.items():
            value = str(value).lower() if isinstance(value, bool) else str(value)
            command += ['-p', f'{key}:={value}']
        return self.start(name, command, essential=essential)

    def check_stack(self):
        for name, process, essential in self.processes:
            if essential and process.poll() is not None:
                raise RuntimeError(
                    f'{name} завершился с кодом {process.returncode}; см. logs/{name}.log')

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.check_stack()
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Таймаут: {description}')
            time.sleep(0.2)

    def wait_for_navigation(self, names=None, manager_groups=None):
        # Сначала проверяем реальные ответы сервисов, затем один раз запускаем manager.
        # Существование action после configure ещё не означает готовность движения.
        import rclpy
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.srv import ManageLifecycleNodes
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger

        context = Context()
        os.environ['ROS_LOG_DIR'] = self.env['ROS_LOG_DIR']
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        rclpy.init(args=[], context=context, domain_id=self.args.domain)
        node = rclpy.create_node('system_startup', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        if names is None:
            names = ['map_server', 'amcl', 'planner_server', 'controller_server']
        if manager_groups is None:
            manager_groups = [
                ('lifecycle_localization', ['map_server', 'amcl']),
                ('lifecycle_navigation', ['planner_server', 'controller_server']),
            ]
        managed_names = [name for _, group in manager_groups for name in group]
        if sorted(managed_names) != sorted(names):
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)
            raise ValueError('Каждый узел Nav2 должен принадлежать одному lifecycle manager')
        state_clients = {name: node.create_client(GetState, f'{name}/get_state') for name in names}
        health_clients = {manager: node.create_client(Trigger, f'{manager}/is_active')
                          for manager, _ in manager_groups}
        startup_clients = {
            manager: node.create_client(ManageLifecycleNodes, f'{manager}/manage_nodes')
            for manager, _ in manager_groups}
        reads = {('state', name): client for name, client in state_clients.items()}
        reads.update({('health', name): client for name, client in health_clients.items()})
        pending = {}
        last_sent = {}
        last_reply = {}
        states = {}
        healthy = {}
        deadline = time.monotonic() + 60

        def poll_read_requests():
            for key, client in reads.items():
                if key in pending:
                    future, sent_at = pending[key]
                    if future.done():
                        del pending[key]
                        if future.exception() is not None:
                            last_reply.pop(key, None)
                            continue
                        response = future.result()
                        if key[0] == 'state':
                            states[key[1]] = response.current_state.id
                        else:
                            healthy[key[1]] = response.success
                        last_reply[key] = time.monotonic()
                    elif time.monotonic() - sent_at > 3:
                        client.remove_pending_request(future)
                        future.cancel()
                        del pending[key]
                        last_reply.pop(key, None)
                elif client.service_is_ready() and time.monotonic() - last_sent.get(key, 0) > 0.25:
                    request = GetState.Request() if key[0] == 'state' else Trigger.Request()
                    pending[key] = (client.call_async(request), time.monotonic())
                    last_sent[key] = time.monotonic()

        def pump():
            self.check_stack()
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Таймаут старта Nav2: состояния={states}, managers={healthy}')
            executor.spin_once(timeout_sec=0.05)
            poll_read_requests()

        try:
            stable_since = None
            while True:
                pump()
                current = time.monotonic()
                replies_fresh = all(current - last_reply.get(key, 0) < 1 for key in reads)
                unconfigured = all(states.get(name) == State.PRIMARY_STATE_UNCONFIGURED
                                   for name in names)
                managers_waiting = all(healthy.get(manager) is False
                                       for manager, _ in manager_groups)
                startup_ready = all(client.service_is_ready()
                                    for client in startup_clients.values())
                if replies_fresh and unconfigured and managers_waiting and startup_ready:
                    if stable_since is None:
                        stable_since = current
                    if current - stable_since >= 2:
                        break
                else:
                    stable_since = None

            # Это не повтор переходов: каждый manager получает ровно один STARTUP.
            # Его ответ и GetState/health после запуска подтверждают activation и bonds.
            for manager, group in manager_groups:
                request = ManageLifecycleNodes.Request()
                request.command = ManageLifecycleNodes.Request.STARTUP
                client = startup_clients[manager]
                sent_at = time.monotonic()
                future = client.call_async(request)
                while not future.done():
                    pump()
                    if time.monotonic() - sent_at > 30:
                        client.remove_pending_request(future)
                        future.cancel()
                        raise RuntimeError(f'{manager}: нет ответа STARTUP за 30 секунд')
                if future.exception() is not None or not future.result().success:
                    raise RuntimeError(f'{manager}: STARTUP завершился ошибкой; см. log Nav2')
                while not (all(states.get(name) == State.PRIMARY_STATE_ACTIVE and
                               last_reply.get(('state', name), 0) >= sent_at for name in group) and
                           healthy.get(manager) is True and
                           last_reply.get(('health', manager), 0) >= sent_at):
                    pump()
                print(f'Nav2 active: {manager}', flush=True)
        finally:
            for key, (future, _) in pending.items():
                reads[key].remove_pending_request(future)
                future.cancel()
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)

    def save_provenance(self):
        folders = ['simulation/integration', 'ros2_ws/src/capstone_mission',
                   'ros2_ws/src/capstone_nav2', 'ros2_ws/src/capstone_safety',
                   'ros2_ws/src/capstone_webots_bridge', 'ros2_ws/src/capstone_system_test',
                   'ros2_ws/src/capstone_benchmark']
        sources = [path for folder in folders for path in (ROOT / folder).rglob('*')
                   if path.is_file() and '__pycache__' not in path.parts]
        sources.append(Path(__file__).resolve())
        digest = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in sorted(sources)}
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True)
        (self.output / 'provenance.json').write_text(json.dumps({
            'scope': 'Webots physics + encoder odometry + lidar AMCL + Nav2 + G5 + safety',
            'method': self.args.method, 'mode': self.args.mode, 'code_commit': commit,
            'worktree_dirty': bool(dirty),
            'ros_distro': self.env.get('ROS_DISTRO', ''), 'ros_domain_id': self.args.domain,
            'webots': 'R2025a', 'source_sha256': digest,
            'command': sys.argv,
        }, indent=2) + '\n')
        return commit

    def run(self):
        commit = self.save_provenance()
        world = ROOT / 'simulation/integration/warehouse.wbt'
        scenario = ROOT / 'ros2_ws/src/capstone_system_test/scenarios/deliveries.yaml'
        events = self.output / 'events.csv'
        port = str(12400 + self.args.domain)
        webots_env = self.env.copy()
        webots_env['WEBOTS_ORIGINAL_LD_LIBRARY_PATH'] = webots_env.get('LD_LIBRARY_PATH', '')
        webots_env['WEBOTS_TMPDIR'] = '/tmp'
        webots_env['LD_LIBRARY_PATH'] = (
            str(self.webots / 'lib/webots') + ':' + webots_env.get('LD_LIBRARY_PATH', ''))
        webots_env['QT_QPA_PLATFORM'] = 'xcb'
        command = [self.webots / 'bin/webots-bin', '--batch', '--mode=realtime',
                   '--stdout', '--stderr', f'--port={port}', world]
        if not self.args.gui:
            if not shutil.which('xvfb-run', path=self.env['PATH']):
                raise RuntimeError('Нет Xvfb; выполните make g5-system-setup')
            command = ['xvfb-run', '-a', '-s', '-screen 0 1024x768x24',
                       *command[:-1], '--no-rendering', world]
        self.start('webots', command, env=webots_env)
        self.start('bridge', [
            self.webots / 'webots-controller', '--robot-name=g5_robot', f'--port={port}',
            self.executable('capstone_webots_bridge', 'webots_bridge')])
        self.start('nav2', ['ros2', 'launch', 'capstone_nav2', 'navigation.launch.py',
                            f'map:={ROOT / "simulation/integration/map.yaml"}',
                            'autostart:=false'])
        self.wait_for_navigation()
        self.start('diagnostics', ['ros2', 'topic', 'echo', 'diagnostics',
                                   'diagnostic_msgs/msg/DiagnosticArray', '--csv'])
        repeated = 2 if self.args.mode == 'repeat' else 1
        evaluator = self.node('evaluator', 'capstone_system_test', 'system_evaluator', {
            'use_sim_time': True, 'output_directory': self.output / 'physical',
            'wall_timeout': self.args.timeout * repeated, 'required_episodes': repeated,
            'expected_terminal': 'cancelled' if self.args.mode == 'cancel' else 'completed',
            'cancel_after_distance': 1.0 if self.args.mode == 'cancel' else -1.0,
            'require_detour': self.args.mode != 'cancel',
            'event_log': events, 'manifest_path': scenario,
        }, essential=False)
        self.node('mission', 'capstone_mission', 'mission_node', {
            'use_sim_time': True, 'autostart': False, 'require_safety': True,
            'manifest_path': scenario, 'event_log': events, 'method': self.args.method,
            'model_path': ROOT / 'ros2_ws/src/capstone_mission/models/dispatch_dqn.json',
            'planning_timeout': 2.0, 'readiness_timeout': 60.0,
        })
        for episode in range(1, repeated + 1):
            run_dir = self.output / f'episode_{episode}'
            runner_params = {
                'use_sim_time': True, 'run_episode': True, 'output_directory': run_dir,
                'simulation_reset_service': 'simulation/reset', 'startup_timeout': 60.0,
                'service_settle_time': 2.0,
                'horizon': 240.0, 'method': self.args.method, 'code_commit': commit,
            }
            runner = self.node(f'runner_{episode}', 'capstone_benchmark', 'benchmark_runner_node',
                               runner_params, essential=False)
            self.wait_until(lambda: runner.poll() is not None or
                            (evaluator.poll() is not None and evaluator.returncode != 0),
                            self.args.timeout, 'завершение миссии')
            if runner.poll() is None:
                raise RuntimeError(
                    'Evaluator завершил проверку раньше runner; см. physical/audit.yaml')
            episode_data = yaml.safe_load((run_dir / 'episode.yaml').read_text())
            expected = 'mission_cancelled' if self.args.mode == 'cancel' else 'completed'
            if episode_data['terminal_reason'] != expected:
                raise RuntimeError(
                    f'Миссия: {episode_data["terminal_reason"]}; см. logs и events.csv')
            evidence = self.output / 'physical' / f'episode_{episode}.yaml'
            self.wait_until(lambda: evidence.exists() or evaluator.poll() is not None, 15,
                            'подтверждение физической остановки')
            if not evidence.exists():
                raise RuntimeError('Физическая проверка не прошла; см. physical/audit.yaml')
            if self.args.mode != 'cancel':
                with (self.output / 'logs' / f'metrics_{episode}.log').open('w') as stream:
                    subprocess.run([self.executable('capstone_benchmark', 'mission_metrics'),
                                    str(scenario), str(events), str(run_dir / 'episode.yaml'),
                                    str(run_dir / 'metrics.csv')], env=self.env, check=True,
                                   stdout=stream, stderr=subprocess.STDOUT)
            print(f'Эпизод {episode}: {expected}, физическое движение и остановка подтверждены.',
                  flush=True)
        self.wait_until(lambda: evaluator.poll() is not None, 15, 'итоговый audit')
        if evaluator.returncode != 0:
            raise RuntimeError('Физическая проверка не прошла; см. physical/audit.yaml')
        print(f'PASS: {self.output}', flush=True)

    def close(self):
        # Сначала даём ROS/контроллеру обработать SIGINT, потом ограничиваем ожидание.
        for _, process, _ in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        deadline = time.monotonic() + 8
        for _, process, _ in reversed(self.processes):
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
        time.sleep(0.2)
        for _, process, _ in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in self.logs:
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--method', choices=['classical', 'learned'], default='classical')
    parser.add_argument('--mode', choices=['delivery', 'cancel', 'repeat'], default='delivery')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'results' / time.strftime('g5-system-%Y%m%d-%H%M%S'))
    parser.add_argument('--domain', type=int, default=185)
    parser.add_argument('--timeout', type=float, default=360.0)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.domain <= 220 or args.timeout <= 0:
        parser.error('domain: 1..220, timeout > 0')
    run = SystemRun(args)
    try:
        run.run()
        return 0
    except (RuntimeError, OSError, subprocess.CalledProcessError, KeyboardInterrupt) as error:
        print(f'FAIL: {error}\nРезультаты: {run.output}', file=sys.stderr)
        return 1
    finally:
        run.close()


if __name__ == '__main__':
    sys.exit(main())
