"""Один воспроизводимый эпизод G5 с C++ runner и явным кодом неуспеха."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    mission = get_package_share_directory('capstone_mission')
    output = LaunchConfiguration('output')
    manifest = LaunchConfiguration('manifest')
    events = PathJoinSubstitution([output, 'events.csv'])
    episode = PathJoinSubstitution([output, 'episode.yaml'])
    arguments = [
        DeclareLaunchArgument('output', default_value='/tmp/g5-debug-episode'),
        DeclareLaunchArgument(
            'manifest', default_value=os.path.join(mission, 'config', 'demo.yaml')),
        DeclareLaunchArgument('method', default_value='classical'),
        DeclareLaunchArgument('model', default_value=os.path.join(
            mission, 'models', 'dispatch_dqn.json')),
        DeclareLaunchArgument('domain_id', default_value='175'),
        DeclareLaunchArgument('commit', default_value='unrecorded'),
        DeclareLaunchArgument('time_scale', default_value='5.0'),
        DeclareLaunchArgument('blocked_route', default_value='false'),
        DeclareLaunchArgument('controller_failure', default_value='false'),
    ]
    simulation = Node(
        package='capstone_debug_navigation', executable='debug_navigation', output='screen',
        parameters=[{'time_scale': LaunchConfiguration('time_scale'),
                     'blocked_route': LaunchConfiguration('blocked_route'),
                     'controller_failure': LaunchConfiguration('controller_failure')}])
    mission_node = Node(
        package='capstone_mission', executable='mission_node', output='screen', parameters=[{
            'use_sim_time': True, 'autostart': False, 'require_safety': False,
            'manifest_path': manifest, 'event_log': events,
            'method': LaunchConfiguration('method'), 'model_path': LaunchConfiguration('model'),
        }])
    runner = Node(
        package='capstone_benchmark', executable='benchmark_runner_node', output='screen',
        parameters=[{'run_episode': True, 'use_sim_time': True, 'output_directory': output,
                     'method': LaunchConfiguration('method'),
                     'code_commit': LaunchConfiguration('commit')}])
    metrics = ExecuteProcess(cmd=[
        PathJoinSubstitution([FindPackagePrefix('capstone_benchmark'), 'lib',
                              'capstone_benchmark', 'mission_metrics']),
        manifest, events, episode, PathJoinSubstitution([output, 'metrics.csv'])], output='screen')
    result = {'runner_code': 1}

    def on_runner_exit(event, context):
        del context
        result['runner_code'] = event.returncode
        return [metrics]

    def on_metrics_exit(event, context):
        del context
        if event.returncode != 0 or result['runner_code'] != 0:
            raise RuntimeError('G5 episode failed; inspect episode.yaml and events.csv')
        return [EmitEvent(event=Shutdown(reason='G5 episode and metrics completed'))]

    return LaunchDescription(arguments + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=on_runner_exit)),
        RegisterEventHandler(OnProcessExit(target_action=metrics, on_exit=on_metrics_exit)),
        simulation, mission_node, runner,
    ])
