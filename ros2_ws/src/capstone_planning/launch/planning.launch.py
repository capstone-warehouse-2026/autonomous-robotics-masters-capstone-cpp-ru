"""G3 planning node with config/planning.yaml; method can be overridden.

ros2 launch capstone_planning planning.launch.py method:=classical
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('capstone_planning'), 'config', 'planning.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='classical', description='classical or learned'),
        Node(
            package='capstone_planning',
            executable='planning_node',
            name='planning',
            output='screen',
            parameters=[
                config,
                {
                    'method': LaunchConfiguration('method'),
                    'use_sim_time': True,
                },
            ],
        ),
    ])
