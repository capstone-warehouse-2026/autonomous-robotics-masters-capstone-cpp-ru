#!/usr/bin/env bash
# Record an already-running, authorized G1 session. Does NOT launch Webots.
set -euo pipefail
if [[ $# != 2 ]]; then echo 'usage: record.sh OUTPUT_BAG DURATION_SECONDS' >&2; exit 2; fi
[[ ! -e "$1" ]] || { echo 'Output exists; refusing overwrite' >&2; exit 2; }
[[ "$2" =~ ^[1-9][0-9]*$ ]] || { echo 'Duration must be a positive integer' >&2; exit 2; }
# Sensor best-effort and transient-local map/static TF are declared explicitly.
qos=$(mktemp)
trap 'rm -f "$qos"' EXIT
cat > "$qos" <<'YAML'
/sensors/wheel_odom: {reliability: best_effort, durability: volatile}
/sensors/imu: {reliability: best_effort, durability: volatile}
/sensors/scan: {reliability: best_effort, durability: volatile}
/map: {reliability: reliable, durability: transient_local}
/tf_static: {reliability: reliable, durability: transient_local}
YAML
set +e
timeout --signal=INT --kill-after=15s "$2" ros2 bag record -o "$1" --qos-profile-overrides-path "$qos" \
 /clock /sensors/wheel_odom /sensors/imu /sensors/scan /map /tf_static /evaluation/ground_truth
status=$?
set -e
[[ $status == 0 || $status == 124 ]] || exit "$status"
test -s "$1/metadata.yaml"
ros2 bag info "$1"
