#!/usr/bin/env python3
"""Проверить CBS/ECBS и распределение заявок на настоящих роботах Webots."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from run_g5_system import ROOT, SystemRun
import yaml


class FleetRun(SystemRun):
    def save_provenance(self):
        folders = ['simulation/fleet', 'ros2_ws/src/capstone_fleet',
                   'ros2_ws/src/capstone_fleet_runtime', 'ros2_ws/src/capstone_nav2',
                   'ros2_ws/src/capstone_webots_bridge', 'ros2_ws/src/capstone_safety']
        sources = [path for folder in folders for path in (ROOT / folder).rglob('*')
                   if path.is_file() and '__pycache__' not in path.parts]
        sources += [Path(__file__).resolve(), ROOT / 'scripts/evaluate_fleet.py',
                    ROOT / 'scripts/fleet_evidence.py', ROOT / 'scripts/run_g5_system.py',
                    ROOT / 'scripts/run_fleet_system.sh']
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        versions = {}
        for package in ['nav2_controller', 'nav2_amcl']:
            import xml.etree.ElementTree as tree
            manifest = Path('/opt/ros/jazzy/share') / package / 'package.xml'
            versions[package] = tree.parse(manifest).getroot().findtext('version')
        (self.output / 'provenance.json').write_text(json.dumps({
            'scope': 'multi_robot_Webots_physics_Nav2_CBS_ECBS_dispatch',
            'code_commit': commit, 'worktree_dirty': bool(subprocess.check_output(
                ['git', 'status', '--porcelain'], cwd=ROOT, text=True)),
            'method': self.args.method, 'weight': self.args.weight,
            'ros_domain_id': self.args.domain, 'webots': 'R2025a', 'versions': versions,
            'command': sys.argv,
            'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(
                path.read_bytes()).hexdigest() for path in sorted(sources)},
        }, indent=2) + '\n')

    def run(self):
        scenario_path = self.args.manifest.resolve()
        scenario = yaml.safe_load(scenario_path.read_text())
        robots = scenario['robots']
        if len(robots) != 2 or {robot['id'] for robot in robots} != {'robot_1', 'robot_2'}:
            raise RuntimeError(
                'Этот физический мир содержит robot_1 и robot_2; нужен их сценарий.')
        stations = {station['id']: station for station in scenario['stations']}
        self.save_provenance()
        (self.output / 'scenario.yaml').write_text(scenario_path.read_text())
        world = ROOT / 'simulation/fleet/warehouse.wbt'
        map_path = ROOT / 'simulation/fleet/map.yaml'
        port = str(12400 + self.args.domain)
        webots_env = self.env.copy()
        webots_env['WEBOTS_ORIGINAL_LD_LIBRARY_PATH'] = webots_env.get('LD_LIBRARY_PATH', '')
        webots_env['LD_LIBRARY_PATH'] = (
            str(self.webots / 'lib/webots') + ':' + webots_env.get('LD_LIBRARY_PATH', ''))
        webots_env['WEBOTS_TMPDIR'] = '/tmp'
        webots_env['QT_QPA_PLATFORM'] = 'xcb'
        command = [self.webots / 'bin/webots-bin', '--batch', '--mode=realtime',
                   '--stdout', '--stderr', f'--port={port}']
        if not self.args.gui:
            command = ['xvfb-run', '-a', '-s', '-screen 0 1024x768x24', *command, '--no-rendering']
        self.start('webots', [*command, world], env=webots_env)
        for index, robot in enumerate(robots):
            name = robot['id']
            station = stations[robot['initial_station']]
            self.start(name + '_bridge', [
                self.webots / 'webots-controller', f'--robot-name={name}', f'--port={port}',
                self.executable('capstone_webots_bridge', 'webots_bridge'), '--ros-args',
                '-r', f'__ns:=/{name}', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
                '-r', 'clock:=/clock', '-p', f'publish_clock:={str(index == 0).lower()}',
                '-p', 'absolute_time:=true', '-p', 'enable_reset:=false'])
            self.start(name + '_nav2', [
                'ros2', 'launch', 'capstone_nav2', 'fleet_robot.launch.py',
                f'namespace:={name}', f'map:={map_path}',
                f'initial_x:={station["x"]}', f'initial_y:={station["y"]}',
                f'initial_yaw:={station.get("yaw", 0.0)}', 'autostart:=false'])
        names = [f'{robot["id"]}/{node}' for robot in robots
                 for node in ['map_server', 'amcl', 'planner_server', 'controller_server']]
        manager_groups = []
        for robot in robots:
            group = [name for name in names if name.startswith(robot['id'] + '/')]
            manager_groups.append((f'{robot["id"]}/lifecycle_fleet', group))
        self.wait_for_navigation(names, manager_groups)
        for robot in robots:
            self.start(robot['id'] + '_diagnostics', [
                'ros2', 'topic', 'echo', f'/{robot["id"]}/diagnostics',
                'diagnostic_msgs/msg/DiagnosticArray', '--csv'])
        evaluator = self.start('evaluator', [
            '/usr/bin/python3', ROOT / 'scripts/evaluate_fleet.py',
            '--manifest', scenario_path, '--map', map_path,
            '--output', self.output / 'physical', '--runtime-output', self.output / 'runtime',
            '--timeout', str(self.args.timeout), '--mode', self.args.mode], essential=False)
        self.wait_until(lambda: (self.output / 'physical/ready').exists() or
                        evaluator.poll() is not None, 20, 'приём физических измерений evaluator')
        if evaluator.poll() is not None:
            raise RuntimeError('Evaluator не смог начать сбор измерений; см. logs/evaluator.log')
        coordinator = self.node('coordinator', 'capstone_fleet_runtime', 'fleet_coordinator', {
            'use_sim_time': True, 'manifest_path': scenario_path,
            'output_directory': self.output / 'runtime', 'method': self.args.method,
            'weight': self.args.weight,
        }, essential=False)
        self.wait_until(lambda: coordinator.poll() is not None or
                        (evaluator.poll() is not None and evaluator.returncode != 0),
                        self.args.timeout, 'миссия нескольких роботов')
        if coordinator.poll() is None:
            raise RuntimeError('Физический evaluator отверг прогон; см. physical/audit.yaml')
        self.wait_until(lambda: evaluator.poll() is not None, 15, 'физическая остановка флота')
        if evaluator.returncode != 0:
            raise RuntimeError('Физическая проверка флота не прошла; см. physical/audit.yaml')
        if self.args.mode == 'delivery' and coordinator.returncode != 0:
            raise RuntimeError('Координатор завершился с ошибкой; см. logs/coordinator.log')
        print(f'PASS {self.args.method.upper()}: {self.output}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--method', choices=['cbs', 'ecbs'], default='cbs')
    parser.add_argument('--weight', type=float, default=1.5)
    parser.add_argument('--mode', choices=['delivery', 'cancel'], default='delivery')
    parser.add_argument('--manifest', type=Path, default=ROOT /
                        'ros2_ws/src/capstone_fleet_runtime/scenarios/swap.yaml')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'results' / time.strftime('fleet-%Y%m%d-%H%M%S'))
    parser.add_argument('--domain', type=int, default=193)
    parser.add_argument('--timeout', type=float, default=480.0)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.domain <= 220 or args.timeout <= 0 or not 1 <= args.weight <= 5:
        parser.error('domain: 1..220, timeout > 0, weight: 1..5')
    run = FleetRun(args)
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
