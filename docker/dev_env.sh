# Environment of the development container (sourced by interactive shells and by make targets).
# shellcheck shell=bash disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi
if [ -n "${PS1:-}" ]; then
  PS1='(capstone-dev) \w\$ '
fi
