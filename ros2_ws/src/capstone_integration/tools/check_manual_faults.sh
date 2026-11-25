#!/usr/bin/env bash
# Two faults that the in-process injector cannot produce by itself:
#   - broken learned model: perception must survive and fall back to the classical method;
#   - frozen /clock: the simulator is paused while the robot moves; the safety monitor must stop it.
#   tools/check_manual_faults.sh [results_dir]
# Both probes run with `docker exec` inside the live stack, so TF and DDS are already up.
# Writes both rows into results/faults/faults.csv. Needs a free simulator (not during a series).
set -euo pipefail
cd "$(dirname "$0")/../../../.."

results="${1:-results}"
faults_dir="$results/faults"
csv="$faults_dir/faults.csv"
mkdir -p "$faults_dir"
if [ ! -f "$csv" ]; then
  echo "run_id,fault,stop_latency_s,diag_latency_s,diag_reason,recovered,notes" > "$csv"
fi

sim_container="$(docker compose ps -q simulator | head -n 1)"
if [ -z "$sim_container" ]; then
  sim_container="autonomous-robotics-masters-capstone-cpp-ru-simulator-1"
fi

exec_in_stack() {
  docker exec clockfreeze bash -c "source /etc/capstone_dev.sh && $1"
}

# `--field` is unreliable inside the stack container; parse the first linear.x from the message.
probe_speed() {
  exec_in_stack 'timeout 6 ros2 topic echo /cmd_vel --once' 2>/dev/null \
    | awk '/x:/ {print $2; exit}'
}

echo "=== starting the stack (perception off, mission on) ==="
docker compose up -d simulator >/dev/null
sleep 25
docker rm -f clockfreeze >/dev/null 2>&1 || true
docker compose run -d --name clockfreeze dev bash -c \
  "source /etc/capstone_dev.sh && ros2 launch capstone_integration system.launch.py start_perception:=false episode_id:=clockfreeze" >/dev/null

echo "=== waiting for a non-zero /cmd_vel ==="
before=""
for _ in $(seq 1 20); do
  before="$(probe_speed)"
  if [ -n "$before" ] && awk -v v="$before" 'BEGIN { exit !(v > 0.1) }'; then
    break
  fi
  sleep 1
done

echo "=== broken model: perception with a missing ONNX file ==="
model_log="$(exec_in_stack \
  "timeout 20 ros2 run capstone_perception perception_node --ros-args -p method:=learned -p learned.model_path:=/nonexistent.onnx -p use_sim_time:=true" 2>&1 || true)"
fallbacks="$(printf '%s' "$model_log" | grep -c 'classical fallback' || true)"
if [ "${fallbacks:-0}" -gt 0 ]; then
  echo "broken_model,broken_model,NA,NA,classical fallback,true,node survived; classical fallback engaged $fallbacks times" >> "$csv"
  echo "PASS: classical fallback engaged ($fallbacks log lines)"
else
  echo "broken_model,broken_model,NA,NA,missing,false,no classical fallback observed" >> "$csv"
  echo "FAIL: no classical fallback observed"
fi

echo "=== frozen clock: pause the simulator while the robot moves ==="
log_lines="$(docker logs clockfreeze 2>&1 | wc -l)"
docker pause "$sim_container" >/dev/null
sleep 4
during="$(probe_speed)"
new_halt="$(docker logs clockfreeze 2>&1 | tail -n +$((log_lines + 1)) | grep -o 'halt: [A-Z_]*' | tail -n 1 || true)"
docker unpause "$sim_container" >/dev/null
sleep 4
after="$(probe_speed)"

stopped=false
if [ -n "$during" ] && awk -v v="$during" 'BEGIN { exit !(v <= 0.1) }'; then
  stopped=true
fi
recovered=false
if [ "$stopped" = true ] && [ -n "$after" ] && awk -v v="$after" 'BEGIN { exit !(v > 0.1) }'; then
  recovered=true
fi
if [ "$stopped" = true ]; then
  note="cmd_vel before=$before during=$during after=$after"
  echo "PASS: the robot stopped while /clock was frozen"
else
  note="NOT DETECTED: cmd_vel stayed $during during the pause; halt in the window=${new_halt:-none}; before=$before after=$after"
  echo "FAIL: /cmd_vel kept a non-zero command while /clock was frozen"
fi
echo "clock_freeze,clock_freeze,NA,NA,${new_halt:-none},$recovered,$note" >> "$csv"
echo "clock freeze: before=$before during=$during after=$after reason=${new_halt:-none}"

docker rm -f clockfreeze >/dev/null 2>&1 || true
docker compose stop simulator >/dev/null
