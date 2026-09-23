"""Start Webots with the smoke world and publish /clock through Ros2Supervisor.

Runs with `ros2 launch` or directly with `python3`; the direct form exits non-zero
when Webots stops or prints an ERROR line (broken world, unknown PROTO, missing asset),
because Webots keeps simulating a partially loaded scene after such errors.
SIM_GUI=1 opens the Webots window, otherwise Webots renders without a window.
"""

import os
import sys

import launch
from launch.substitutions import EnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds')

_state = {'exit_code': None, 'error': None}


def _on_webots_exit(event, _context):
    _state['exit_code'] = event.returncode
    return [launch.actions.Shutdown(reason=f'Webots exited with code {event.returncode}')]


def _on_webots_output(event):
    if _state['error'] is not None:
        return None
    for line in event.text.decode(errors='replace').splitlines():
        if line.lstrip().startswith('ERROR:'):
            _state['error'] = line.strip()
            return [launch.actions.Shutdown(reason=f'Webots reported an error: {_state["error"]}')]
    return None


def generate_launch_description():
    webots = WebotsLauncher(
        world=os.path.join(WORLDS_DIR, 'smoke.wbt'),
        gui=EnvironmentVariable('SIM_GUI', default_value='0'),
        mode='realtime',
        ros2_supervisor=True,
    )
    return launch.LaunchDescription([
        webots,
        webots._supervisor,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessIO(
                target_action=webots, on_stdout=_on_webots_output, on_stderr=_on_webots_output)
        ),
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(target_action=webots, on_exit=_on_webots_exit)
        ),
    ])


if __name__ == '__main__':
    service = launch.LaunchService()
    service.include_launch_description(generate_launch_description())
    launch_status = service.run()
    if _state['error'] is not None:
        print(f'FAIL: Webots could not load the world cleanly: {_state["error"]}', file=sys.stderr)
        sys.exit(1)
    exit_code = _state['exit_code']
    if exit_code is not None and exit_code < 0:
        # Stopped by a signal (Ctrl+C, docker stop): report it the way a shell does.
        sys.exit(128 - exit_code)
    if exit_code == 0 and os.environ.get('SIM_GUI', '0') == '1':
        sys.exit(0)  # the user closed the Webots window
    # A headless simulation never ends by itself: any other Webots exit is a failure.
    sys.exit(exit_code or launch_status or 1)
