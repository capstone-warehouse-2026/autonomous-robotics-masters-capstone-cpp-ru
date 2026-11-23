#!/usr/bin/env bash
set -eo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
mode="${1:-mock}"
if [[ "$mode" != mock && "$mode" != real ]]; then
  echo "Usage: $0 [mock|real]" >&2
  exit 2
fi
distro="${G1_ROS_DISTRO:-jazzy}"
case "$distro" in
  jazzy)
    unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ;;
  humble)
    unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
    export ROS_LOCALHOST_ONLY=1
    ;;
  *) echo "G1_ROS_DISTRO must be jazzy (course) or humble (prototype)" >&2; exit 2 ;;
esac
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$distro" ]]; then
  echo "Start from a fresh shell; ROS_DISTRO=$ROS_DISTRO, expected $distro" >&2
  exit 2
fi
export ROS_DOMAIN_ID="${G1_SMOKE_DOMAIN_ID:-$((170 + RANDOM % 55))}"
if [[ ! "$ROS_DOMAIN_ID" =~ ^(17[0-9]|18[0-9]|19[0-9]|20[0-9]|21[0-9]|22[0-4])$ ]]; then
  echo "G1_SMOKE_DOMAIN_ID must be an unused domain in 170..224" >&2
  exit 2
fi
setup="/opt/ros/$distro/setup.bash"
if [[ ! -r "$setup" ]]; then
  echo "$setup not found. For course Jazzy use bash integration/g1_v1/course_smoke.sh mock." >&2
  exit 2
fi
source "$setup"
pids=()
cleanup() {
  if (( ${#pids[@]} )); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "G1 smoke: $distro mode=$mode, local ROS domain=$ROS_DOMAIN_ID"
if [[ "$mode" == real ]]; then
  overlay="${G1_INSTALL_PREFIX:-$here/../../ros2_ws/install}"
  source "$overlay/local_setup.bash"
  executable="$(ros2 pkg prefix capstone_localization)/lib/capstone_localization/localization_node"
  "$executable" --ros-args -p use_sim_time:=true -p method:=classical &
  pids+=("$!")
  mode=sensors
fi
python3 "$here/fixture.py" "$mode" &
pids+=("$!")
python3 "$here/fixture.py" check
