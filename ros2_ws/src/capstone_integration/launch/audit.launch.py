"""Contract audit against a running system (read-only).

    make sim-gui            # terminal 1
    make shell              # terminal 2, start the stack
    ros2 launch capstone_integration system.launch.py

    # terminal 3, same container image
    ros2 run capstone_integration graph_audit --ros-args -p window_s:=5.0 -p fail_on:=error

Exit code is 1 when a FAIL was found and fail_on=error (0 for fail_on=never).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('window_s', default_value='5.0',
                              description='sampling window in seconds'),
        DeclareLaunchArgument('settle_s', default_value='2.0',
                              description='wait before sampling so discovery completes'),
        DeclareLaunchArgument('fail_on', default_value='error',
                              description='never | warn | error: when the exit code is 1'),
        DeclareLaunchArgument('measured_run', default_value='false',
                              description='true: placeholder nodes make the audit FAIL'),
        DeclareLaunchArgument('json_path', default_value='',
                              description='write the report to this file inside the container'),
        Node(
            package='capstone_integration',
            executable='graph_audit',
            name='graph_audit',
            output='screen',
            parameters=[{
                'window_s': LaunchConfiguration('window_s'),
                'settle_s': LaunchConfiguration('settle_s'),
                'fail_on': LaunchConfiguration('fail_on'),
                'measured_run': LaunchConfiguration('measured_run'),
                'json_path': LaunchConfiguration('json_path'),
            }],
        ),
    ])
