"""G4 safety monitor with config/safety.yaml; the only publisher of /cmd_vel.

ros2 launch capstone_safety safety.launch.py
ros2 launch capstone_safety safety.launch.py autostart:=false   # G5 drives lifecycle transitions itself
ros2 launch capstone_safety safety.launch.py use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('capstone_safety'), 'config', 'safety.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='use /clock of the simulator'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='configure+activate on start; false: transitions via lifecycle services'),
        DeclareLaunchArgument('params_file', default_value=config, description='safety thresholds'),
        Node(
            package='capstone_safety',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                },
            ],
        ),
    ])
