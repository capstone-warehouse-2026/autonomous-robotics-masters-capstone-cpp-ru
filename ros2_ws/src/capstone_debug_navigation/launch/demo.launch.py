"""Изолированный стенд G5: кинематика, ROS actions и настоящее дерево миссии."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission = get_package_share_directory('capstone_mission')
    return LaunchDescription([
        DeclareLaunchArgument(
            'manifest', default_value=os.path.join(mission, 'config', 'demo.yaml')),
        DeclareLaunchArgument('method', default_value='classical'),
        DeclareLaunchArgument('model', default_value=os.path.join(
            mission, 'models', 'dispatch_dqn.json')),
        DeclareLaunchArgument('event_log', default_value='/tmp/g5-debug-events.csv'),
        DeclareLaunchArgument('time_scale', default_value='5.0'),
        Node(package='capstone_debug_navigation', executable='debug_navigation',
             parameters=[{'time_scale': LaunchConfiguration('time_scale')}], output='screen'),
        Node(package='capstone_mission', executable='mission_node', output='screen', parameters=[{
            'use_sim_time': True,
            'manifest_path': LaunchConfiguration('manifest'),
            'method': LaunchConfiguration('method'),
            'model_path': LaunchConfiguration('model'),
            'event_log': LaunchConfiguration('event_log'),
            'require_safety': False,
        }]),
    ])
