#!/usr/bin/env bash
# Fault suite: injects one fault while the robot is moving and records the reaction.
#   tools/run_faults.sh [repeats] [results_dir]
# Faults: sensor_dropout, controller_pause (stale command), controller_crash (process exit).
set -euo pipefail
cd "$(dirname "$0")/../../../.."

repeats="${1:-1}"
results="${2:-results}"
faults=(sensor_dropout controller_pause controller_crash)

for fault in "${faults[@]}"; do
  for run in $(seq 1 "$repeats"); do
    id="fault-$fault-$run"
    echo "=== $id: restarting the simulator ==="
    docker compose restart simulator >/dev/null
    sleep 25
    echo "=== $id: injecting ==="
    timeout 30 docker compose run --rm dev bash -c \
      "source /etc/capstone_dev.sh && ros2 launch capstone_integration system.launch.py start_perception:=false start_faults:=true fault:=$fault episode_id:=$id results_dir:=/workspace/$results" \
      | grep -E 'FAULT|halt:|fault |stop latency|episode .* (started|finished)' || true
    echo
  done
done

docker compose stop simulator >/dev/null
echo "=== done: $results/faults/faults.csv ==="
