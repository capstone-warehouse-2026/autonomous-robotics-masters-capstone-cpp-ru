#!/usr/bin/env bash
set -euo pipefail

# Headless by default: Webots runs inside Xvfb with software (Mesa) rendering, so
# camera/depth output does not depend on the host GPU. SIM_GUI=1 needs a host display.
if [ "${SIM_GUI:-0}" != "1" ]; then
  export WEBOTS_OFFSCREEN=1
fi

# capstone_sim: the robot driver plugin built in the simulator-build stage.
# colcon setup scripts read unset variables, so -u is off while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/capstone/sim_ws/install/setup.bash
set -u
exec python3 /opt/capstone/simulation/launch/warehouse.launch.py
