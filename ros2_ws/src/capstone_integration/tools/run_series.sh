#!/usr/bin/env bash
# Host-side episode series: restarts the simulator between episodes and runs the stack.
#   tools/run_series.sh [episodes] [results_dir]
# Needs Docker and a built dev image (make setup). Run from the repository root.
set -euo pipefail
cd "$(dirname "$0")/../../../.."

episodes="${1:-3}"
results="${2:-results}"

for i in $(seq 1 "$episodes"); do
  id="series-$i"
  rm -rf "$results/$id"
  echo "=== $id: restarting the simulator (fresh world, robot at the start pose) ==="
  docker compose restart simulator >/dev/null
  sleep 25
  echo "=== $id: running the integration stack ==="
  timeout 35 docker compose run --rm dev bash -c \
    "source /etc/capstone_dev.sh && ros2 launch capstone_integration system.launch.py start_perception:=false episode_id:=$id results_dir:=/workspace/$results" \
    || true
  if [ ! -f "$results/$id/episode.csv" ]; then
    echo "WARNING: $id produced no episode.csv (mission did not finish in 35 s)"
  fi
done

docker compose stop simulator >/dev/null
echo "=== done: $results ==="
