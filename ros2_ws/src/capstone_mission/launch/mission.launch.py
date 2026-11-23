"""Запуск G5 с реальными интерфейсами навигации; тестовые серверы включаются отдельно."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('capstone_mission')
    config = os.path.join(share, 'config', 'mission.yaml')
    names = {
        'manifest_path': '',
        'method': 'classical',
        'model_path': os.path.join(share, 'models', 'dispatch_dqn.json'),
        'event_log': 'results/mission_events.csv',
        'use_sim_time': 'true',
        'require_safety': 'true',
    }
    declarations = [DeclareLaunchArgument(name, default_value=value)
                    for name, value in names.items()]
    node = Node(
        package='capstone_mission',
        executable='mission_node',
        output='screen',
        parameters=[config, {name: LaunchConfiguration(name) for name in names}],
    )
    return LaunchDescription(declarations + [node])
