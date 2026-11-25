#!/usr/bin/env bash
# Нативная сборка только зависимостей физического стенда.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export WEBOTS_HOME=${WEBOTS_HOME:-"$root/.cache/webots"}
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'Нужен ROS 2 Jazzy в /opt/ros/jazzy.' >&2; exit 1; }
[[ -f "$WEBOTS_HOME/include/controller/c/webots/robot.h" ]] || {
  echo 'Сначала выполните make g5-system-setup (или задайте WEBOTS_HOME).' >&2; exit 1;
}
source /opt/ros/jazzy/setup.bash
for package in nav2_planner nav2_controller nav2_amcl nav2_map_server \
  nav2_regulated_pure_pursuit_controller nav2_navfn_planner; do
  ros2 pkg prefix "$package" >/dev/null || {
    echo 'Установите ROS Nav2: sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup' >&2
    exit 1
  }
done
cd "$root/ros2_ws"
colcon build --symlink-install --parallel-workers 2 --packages-up-to \
  capstone_webots_bridge capstone_nav2 capstone_system_test capstone_benchmark capstone_safety "$@" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
