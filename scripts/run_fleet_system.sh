#!/usr/bin/env bash
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$root/ros2_ws/install/setup.bash"
exec /usr/bin/python3 "$root/scripts/run_fleet_system.py" "$@"
