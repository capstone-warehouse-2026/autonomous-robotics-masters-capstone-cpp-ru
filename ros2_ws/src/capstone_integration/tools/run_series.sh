#!/usr/bin/env bash
# Episode series with a simulator reset, a live contract audit and a per-episode manifest.
#   tools/run_series.sh [episodes] [results_dir] [measured]
# measured=1 marks the run as measured: placeholder nodes make the audit FAIL and the script exits 1.
set -euo pipefail
cd "$(dirname "$0")/../../../.."

episodes="${1:-1}"
results="${2:-results}"
measured="${3:-0}"
stack_timeout="${STACK_TIMEOUT:-300}"

commit="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
image="$(docker image inspect capstone-dev:local --format '{{.Id}}' 2>/dev/null | cut -c8-19 || echo unknown)"
measured_flag=false
[ "$measured" = "1" ] && measured_flag=true
invalid=0

for i in $(seq 1 "$episodes"); do
  id="series-$i"
  docker rm -f "$id" >/dev/null 2>&1 || true
  rm -rf "$results/$id"
  mkdir -p "$results/$id"
  echo "=== $id: restarting the simulator (fresh world) ==="
  docker compose restart simulator >/dev/null
  sleep 25

  echo "=== $id: running the integration stack ==="
  docker compose run -d --name "$id" dev bash -c \
    "source /etc/capstone_dev.sh && ros2 launch capstone_integration system.launch.py start_perception:=true episode_id:=$id results_dir:=/workspace/$results" >/dev/null

  # Audit while the robot is moving: after the episode ends the controller stops publishing
  # /control/cmd_vel_raw, which would show up as a false FAIL.
  sleep 12
  echo "=== $id: auditing the live graph (measured_run=$measured_flag) ==="
  audit_rc=0
  docker compose run --rm dev bash -c \
    "source /etc/capstone_dev.sh && ros2 run capstone_integration graph_audit --ros-args -p window_s:=5.0 -p settle_s:=1.0 -p measured_run:=$measured_flag -p json_path:=/workspace/$results/$id/audit.json -p fail_on:=error" \
    || audit_rc=$?

  csv="$results/$id/episode.csv"
  for _ in $(seq 1 "$stack_timeout"); do
    [ -f "$csv" ] && break
    sleep 1
  done
  if [ ! -f "$csv" ]; then
    echo "WARNING: $id produced no episode.csv within ${stack_timeout} s"
  fi

  failures="$(sed -n 's/.*"failures": \([0-9]*\).*/\1/p' "$results/$id/audit.json" 2>/dev/null | head -n 1)"
  warnings="$(sed -n 's/.*"warnings": \([0-9]*\).*/\1/p' "$results/$id/audit.json" 2>/dev/null | head -n 1)"
  failures="${failures:-NA}"
  warnings="${warnings:-NA}"

  cat > "$results/$id/manifest.json" <<EOF
{
  "episode_id": "$id",
  "config_id": "SKELETON",
  "scenario_id": "S1",
  "measured_run": $measured_flag,
  "commit": "$commit",
  "dev_image_id": "$image",
  "audit": {"exit_code": $audit_rc, "failures": "$failures", "warnings": "$warnings"},
  "created_at": "$(date -Iseconds)"
}
EOF

  if [ ! -f "$results/series_audit.csv" ]; then
    echo "episode_id,measured,failures,warnings,audit_exit" > "$results/series_audit.csv"
  fi
  echo "$id,$measured_flag,$failures,$warnings,$audit_rc" >> "$results/series_audit.csv"

  if [ "$measured" = "1" ] && [ "$audit_rc" -ne 0 ]; then
    echo "INVALID: $id is marked measured but the audit failed; the episode cannot be reported"
    invalid=1
  fi

  docker rm -f "$id" >/dev/null 2>&1 || true
  echo
done

docker compose stop simulator >/dev/null
echo "=== done: $results (audit summary: $results/series_audit.csv) ==="
exit "$invalid"
