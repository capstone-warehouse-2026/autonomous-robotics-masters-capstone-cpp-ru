"""Общая конфигурация автономии; требует настоящих серверов и датчиков G1–G4."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_components(context):
    """Выбор профиля происходит до запуска узлов и остаётся неизменным в эпизоде."""
    bringup = get_package_share_directory('capstone_bringup')
    with open(os.path.join(bringup, 'config', 'profiles.yaml'), encoding='utf-8') as stream:
        profiles = yaml.safe_load(stream)
    methods = profiles[LaunchConfiguration('profile').perform(context)]
    perception = get_package_share_directory('capstone_perception')
    safety = get_package_share_directory('capstone_safety')
    mission = get_package_share_directory('capstone_mission')
    return [
        Node(package='capstone_localization', executable='localization_node',
             parameters=[{'use_sim_time': True, 'method': methods[0]}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            perception, 'launch', 'perception.launch.py')),
            launch_arguments={'method': methods[1], 'target_frame': 'odom'}.items()),
        Node(package='capstone_planning', executable='planning_node',
             parameters=[{'use_sim_time': True, 'method': methods[2]}], output='screen'),
        Node(package='capstone_control', executable='control_node',
             parameters=[{'use_sim_time': True, 'method': methods[3]}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            safety, 'launch', 'safety.launch.py'))),
        Node(package='capstone_mission', executable='mission_node', output='screen',
             parameters=[os.path.join(mission, 'config', 'mission.yaml'), {
                 'use_sim_time': True,
                 'method': methods[4],
                 'manifest_path': LaunchConfiguration('manifest').perform(context),
                 'event_log': LaunchConfiguration('event_log').perform(context),
                 'require_safety': True,
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='C0',
                              choices=['C0', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6']),
        DeclareLaunchArgument('manifest', description='Файл заявок и станций эпизода'),
        DeclareLaunchArgument('event_log', default_value='/tmp/capstone-events.csv'),
        OpaqueFunction(function=launch_components),
    ])
