"""Nav2 серверы для G5: собственное дерево миссий остаётся в capstone_mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory('capstone_nav2'))
    safety = Path(get_package_share_directory('capstone_safety'))
    parameters = LaunchConfiguration('params_file')
    simulated_time = {'use_sim_time': True}
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)

    # Начальная гипотеза локализации — известная точка старта сценария, не runtime GT.
    localization = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        arguments=['--ros-args', '--log-level', 'amcl:=warn'],
        parameters=[parameters, simulated_time, {
            'initial_pose.x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
            'initial_pose.y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
            'initial_pose.yaw': ParameterValue(
                LaunchConfiguration('initial_yaw'), value_type=float),
        }],
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[parameters, simulated_time, {'yaml_filename': LaunchConfiguration('map')}],
    )
    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen',
        parameters=[parameters, simulated_time],
        remappings=[('compute_path_to_pose', 'planning/compute_path')],
    )
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=[parameters, simulated_time],
        remappings=[('follow_path', 'control/follow_path'), ('cmd_vel', 'nav2/cmd_vel')],
    )
    contracts = Node(
        package='capstone_nav2', executable='nav2_contracts', name='nav2_contracts',
        output='screen', parameters=[parameters, simulated_time, {
            'initial_x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
            'initial_y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
            'initial_yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
        }],
    )
    monitor = Node(
        package='capstone_safety', executable='safety_monitor_node', name='safety_monitor',
        output='screen', parameters=[str(safety / 'config/safety.yaml'), simulated_time,
                                     {'robot_radius': 0.28}],
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_localization', output='screen', parameters=[simulated_time, {
            'autostart': autostart, 'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_navigation', output='screen', parameters=[simulated_time, {
            'autostart': autostart, 'node_names': ['planner_server', 'controller_server'],
            'bond_timeout': 4.0,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Абсолютный путь к статической карте YAML'),
        DeclareLaunchArgument('params_file',
                              default_value=str(package / 'config/navigation.yaml')),
        DeclareLaunchArgument('initial_x', default_value='-3.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        map_server, localization, planner, controller, contracts, monitor,
        localization_lifecycle, navigation_lifecycle,
    ])
