"""Integration skeleton: one command for the whole autonomy stack.

Requires a running simulator (`make sim-gui` or `make sim` in another terminal, or the
simulator service of compose.yml). Run inside the development container:

    make shell
    ros2 launch capstone_integration system.launch.py
    ros2 launch capstone_integration system.launch.py start_perception:=false episode_id:=dbg-2

Every node that stands in for a group is marked PLACEHOLDER in its log line and in the episode CSV.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('capstone_integration')
    params = os.path.join(share, 'config', 'system.yaml')
    safety_launch = os.path.join(get_package_share_directory('capstone_safety'), 'launch',
                                 'safety.launch.py')
    perception_launch = os.path.join(get_package_share_directory('capstone_perception'), 'launch',
                                     'perception.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_safety = LaunchConfiguration('start_safety')
    start_perception = LaunchConfiguration('start_perception')
    start_mission = LaunchConfiguration('start_mission')
    autostart_mission = LaunchConfiguration('autostart_mission')
    episode_id = LaunchConfiguration('episode_id')
    results_dir = LaunchConfiguration('results_dir')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='use /clock from the simulator'),
        DeclareLaunchArgument('start_safety', default_value='true',
                              description='start the real G4 safety monitor'),
        DeclareLaunchArgument('start_perception', default_value='true',
                              description='start the real G2 perception node'),
        DeclareLaunchArgument('start_mission', default_value='true',
                              description='start the placeholder mission node'),
        DeclareLaunchArgument('autostart_mission', default_value='true',
                              description='start the episode without an external command'),
        DeclareLaunchArgument('episode_id', default_value='stub-001',
                              description='episode identifier written to the result CSV'),
        DeclareLaunchArgument('results_dir', default_value='/workspace/results',
                              description='directory of the result CSV (inside the container)'),

        # PLACEHOLDER G1: sensors, localization, map and TF.
        Node(
            package='capstone_integration',
            executable='world_stub_node',
            name='world_stub',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        # PLACEHOLDER G3: ComputePathToPose action server.
        Node(
            package='capstone_integration',
            executable='path_stub_node',
            name='path_stub',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        # PLACEHOLDER G4: FollowPath action server publishing /control/cmd_vel_raw.
        Node(
            package='capstone_integration',
            executable='drive_stub_node',
            name='drive_stub',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        # PLACEHOLDER G5: one fixed job and the episode CSV.
        Node(
            package='capstone_integration',
            executable='mission_stub_node',
            name='mission_stub',
            output='screen',
            condition=IfCondition(start_mission),
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'auto_start': autostart_mission,
                'episode_id': episode_id,
                'results_dir': results_dir,
            }],
        ),

        # Real G4 safety monitor: the only publisher of /cmd_vel.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(safety_launch),
            condition=IfCondition(start_safety),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # Real G2 perception: /perception/obstacles in odom.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            condition=IfCondition(start_perception),
            launch_arguments={'method': 'classical', 'target_frame': 'odom'}.items(),
        ),
    ])
