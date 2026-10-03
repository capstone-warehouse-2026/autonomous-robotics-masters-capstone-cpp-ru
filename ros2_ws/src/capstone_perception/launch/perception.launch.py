"""G2 perception node with config/perception.yaml; method and target_frame can be overridden.

ros2 launch capstone_perception perception.launch.py method:=classical target_frame:=base_link
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('capstone_perception'), 'config', 'perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='classical', description='classical or learned'),
        DeclareLaunchArgument('target_frame', default_value='odom', description='frame of /perception/obstacles'),
        Node(
            package='capstone_perception',
            executable='perception_node',
            name='perception',
            output='screen',
            parameters=[
                config,
                {
                    'method': LaunchConfiguration('method'),
                    'target_frame': LaunchConfiguration('target_frame'),
                    'use_sim_time': True,
                },
            ],
        ),
    ])
