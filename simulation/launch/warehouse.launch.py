"""Start Webots with the warehouse world, /clock (Ros2Supervisor) and the robot driver.

Runs with `ros2 launch` or directly with `python3`; the direct form exits non-zero when
Webots or the robot driver stops, or Webots prints an ERROR line (broken world, unknown
PROTO, missing asset), because Webots keeps simulating a partially loaded scene after such
errors. SIM_GUI=1 opens the Webots window, otherwise Webots renders without a window.
SIM_WORLD selects another world file from simulation/worlds (default: warehouse.wbt).
"""

import os
import sys

import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds')

_state = {'exit_code': None, 'error': None}


def _fail(reason):
    if _state['error'] is None:
        _state['error'] = reason
    return [launch.actions.Shutdown(reason=reason)]


def _on_webots_exit(event, _context):
    _state['exit_code'] = event.returncode
    return [launch.actions.Shutdown(reason=f'Webots exited with code {event.returncode}')]


def _on_driver_exit(event, _context):
    if _state['exit_code'] is not None:
        return None  # Webots already stopped; the driver follows it
    return _fail(f'robot driver exited with code {event.returncode}')


def _on_webots_output(event):
    if _state['error'] is not None:
        return None
    for line in event.text.decode(errors='replace').splitlines():
        if line.lstrip().startswith('ERROR:'):
            return _fail(line.strip())
    return None


def generate_launch_description():
    world = os.environ.get('SIM_WORLD', 'warehouse.wbt')
    webots = WebotsLauncher(
        world=os.path.join(WORLDS_DIR, world),
        gui=EnvironmentVariable('SIM_GUI', default_value='0'),
        mode='realtime',
        ros2_supervisor=True,
    )
    robot_driver = WebotsController(
        robot_name='tiago_base',
        parameters=[
            {'robot_description': os.path.join(
                get_package_share_directory('capstone_sim'), 'resource', 'tiago_base.urdf')},
            {'use_sim_time': True},
        ],
        respawn=False,
    )
    return launch.LaunchDescription([
        webots,
        webots._supervisor,
        robot_driver,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessIO(
                target_action=webots, on_stdout=_on_webots_output, on_stderr=_on_webots_output)
        ),
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(target_action=webots, on_exit=_on_webots_exit)
        ),
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(target_action=robot_driver, on_exit=_on_driver_exit)
        ),
    ])


if __name__ == '__main__':
    service = launch.LaunchService()
    service.include_launch_description(generate_launch_description())
    launch_status = service.run()
    if _state['error'] is not None:
        print(f'FAIL: simulation did not start cleanly: {_state["error"]}', file=sys.stderr)
        sys.exit(1)
    exit_code = _state['exit_code']
    if exit_code is not None and exit_code < 0:
        # Stopped by a signal (Ctrl+C, docker stop): report it the way a shell does.
        sys.exit(128 - exit_code)
    if exit_code == 0 and os.environ.get('SIM_GUI', '0') == '1':
        sys.exit(0)  # the user closed the Webots window
    # A headless simulation never ends by itself: any other Webots exit is a failure.
    sys.exit(exit_code or launch_status or 1)
