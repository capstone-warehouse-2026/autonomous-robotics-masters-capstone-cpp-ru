#!/usr/bin/env bash
set -euo pipefail

# Headless by default: Webots runs inside Xvfb with software (Mesa) rendering, so
# camera/depth output does not depend on the host GPU. SIM_GUI=1 needs a host display.
if [ "${SIM_GUI:-0}" != "1" ]; then
  export WEBOTS_OFFSCREEN=1
fi

exec python3 /opt/capstone/simulation/launch/smoke.launch.py
