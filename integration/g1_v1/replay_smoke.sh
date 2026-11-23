#!/usr/bin/env bash
# Record live fixture and verify playback on a separate ROS domain.
set -eo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
case "${ROS_DISTRO:-}" in
  jazzy) unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ;;
  humble) unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; export ROS_LOCALHOST_ONLY=1 ;;
  *) echo "Source Jazzy (course) or Humble (prototype) first." >&2; exit 2 ;;
esac
export ROS_DOMAIN_ID="$((170 + RANDOM % 54))"
dir="$(mktemp -d /tmp/g1-replay.XXXXXXXX)"
fixture=
checker=
cleanup() {
  for pid in "$fixture" "$checker"; do
    if [[ -n "$pid" ]]; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$here/fixture.py" mock &
fixture="$!"
# Late subscription deliberately exercises transient-local map and static TF.
sleep 1
ret=0
timeout --signal=INT --preserve-status 6 ros2 bag record -s sqlite3 -o "$dir/bag" \
  --qos-profile-overrides-path "$here/bag_qos.yaml" \
  /clock /map /tf_static /tf /sensors/wheel_odom /sensors/imu /sensors/scan \
  /sensors/rgb /sensors/depth /sensors/camera_info /state/odom /localization/pose || ret=$?
if [[ "$ret" != 0 && "$ret" != 130 ]]; then exit "$ret"; fi
kill "$fixture"
wait "$fixture" || true
fixture=
ros2 bag info "$dir/bag"
export ROS_DOMAIN_ID="$((ROS_DOMAIN_ID + 1))"
python3 "$here/fixture.py" check &
checker="$!"
timeout --signal=INT 15 ros2 bag play "$dir/bag" --delay 1 \
  --qos-profile-overrides-path "$here/bag_qos.yaml"
wait "$checker"
checker=
echo "Recorded bag: $dir/bag"
