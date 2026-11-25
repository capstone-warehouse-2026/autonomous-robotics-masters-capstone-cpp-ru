"""Изолированный Nav2 одного робота флота в общей физической карте."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def launch_robot(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be one ROS name, for example robot_1')
    initial_x = float(LaunchConfiguration('initial_x').perform(context))
    initial_y = float(LaunchConfiguration('initial_y').perform(context))
    initial_yaw = float(LaunchConfiguration('initial_yaw').perform(context))
    parameters = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'), root_key=namespace,
        convert_types=True, param_rewrites={
            'scan_topic': f'/{namespace}/sensors/scan',
            'odom_topic': f'/{namespace}/state/odom',
            # Вложенные costmap-ноды должны читать датчик именно своего робота.
            'topic': f'/{namespace}/sensors/scan',
            'global_costmap.global_costmap.ros__parameters.static.map_topic': f'/{namespace}/map',
            # RPP завершает геометрическую цель и публикует ноль. Остановку после result
            # отдельно подтверждает fleet barrier по свежей odom, прежде чем дать новый шаг.
            'controller_server.ros__parameters.goal.plugin': 'nav2_controller::SimpleGoalChecker',
            'controller_server.ros__parameters.goal.xy_goal_tolerance': '0.03',
            'controller_server.ros__parameters.goal.yaw_goal_tolerance': '0.10',
            'controller_server.ros__parameters.FollowPath.lookahead_dist': '0.4',
            # Перед движением по короткому ребру завершить разворот: базовые 0.6 рад
            # дают боковую дугу за пределами физического коридора после смены направления.
            'controller_server.ros__parameters.FollowPath.rotate_to_heading_min_angle': '0.15',
        }), allow_substs=True)
    # Имена frame остаются map/odom/base_link; отдельные TF-топики не дают им смешаться.
    transforms = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('clock', '/clock')]
    sim_time = {'use_sim_time': True}
    safety = Path(get_package_share_directory('capstone_safety'))
    safety_parameters = ParameterFile(RewrittenYaml(
        source_file=str(safety / 'config/safety.yaml'), root_key=namespace,
        param_rewrites={'robot_radius': '0.28'}, convert_types=True), allow_substs=True)
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        namespace=namespace, output='screen', remappings=transforms,
        parameters=[parameters, sim_time, {'yaml_filename': LaunchConfiguration('map')}])
    localization = Node(
        package='nav2_amcl', executable='amcl', name='amcl', namespace=namespace,
        output='screen', remappings=transforms,
        arguments=['--ros-args', '--log-level', f'{namespace}.amcl:=warn'],
        parameters=[parameters, sim_time, {
            'initial_pose.x': initial_x, 'initial_pose.y': initial_y,
            'initial_pose.yaw': initial_yaw}])
    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        namespace=namespace, output='screen', parameters=[parameters, sim_time],
        remappings=transforms + [('compute_path_to_pose', 'planning/compute_path')])
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        namespace=namespace, output='screen', parameters=[parameters, sim_time],
        remappings=transforms + [('follow_path', 'control/follow_path'),
                                 ('cmd_vel', 'nav2/cmd_vel')])
    contracts = Node(
        package='capstone_nav2', executable='nav2_contracts', name='nav2_contracts',
        namespace=namespace, output='screen', remappings=transforms,
        parameters=[parameters, sim_time, {
            'initial_x': initial_x, 'initial_y': initial_y, 'initial_yaw': initial_yaw}])
    monitor = Node(
        package='capstone_safety', executable='safety_monitor_node', name='safety_monitor',
        namespace=namespace, output='screen', remappings=transforms + [
            # Существующий safety использует абсолютные контракты однороботного стенда.
            ('/control/cmd_vel_raw', f'/{namespace}/control/cmd_vel_raw'),
            ('/state/odom', f'/{namespace}/state/odom'),
            ('/sensors/scan', f'/{namespace}/sensors/scan'),
            ('/cmd_vel', f'/{namespace}/cmd_vel'),
            ('/diagnostics', f'/{namespace}/diagnostics')],
        parameters=[safety_parameters, sim_time])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_fleet',
        namespace=namespace, output='screen', parameters=[sim_time, {
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'bond_timeout': 4.0,
            'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server']}])
    return [map_server, localization, planner, controller, contracts, monitor, lifecycle]


def generate_launch_description():
    package = Path(get_package_share_directory('capstone_nav2'))
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='robot_1 или robot_2'),
        DeclareLaunchArgument('map', description='Общая статическая карта YAML'),
        DeclareLaunchArgument('params_file',
                              default_value=str(package / 'config/navigation.yaml')),
        DeclareLaunchArgument('initial_x', default_value='-3.0'),
        DeclareLaunchArgument('initial_y', default_value='-3.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=launch_robot),
    ])
