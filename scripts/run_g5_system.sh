#!/usr/bin/env bash
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
[[ -f "$root/ros2_ws/install/setup.bash" ]] || {
  echo 'Сначала выполните make g5-system-build.' >&2; exit 1;
}
source "$root/ros2_ws/install/setup.bash"
exec /usr/bin/python3 "$root/scripts/run_g5_system.py" "$@"
