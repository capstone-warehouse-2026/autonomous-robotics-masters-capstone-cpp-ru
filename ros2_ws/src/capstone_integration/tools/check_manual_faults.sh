#!/usr/bin/env bash
# Two faults that the in-process injector cannot produce by itself:
#   - broken learned model: perception must survive and fall back to the classical method;
#   - frozen /clock: the simulator is paused while the robot moves; the safety monitor must stop it.
#   tools/check_manual_faults.sh [results_dir]
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

echo "=== broken model: perception with a missing ONNX file ==="
docker compose up -d simulator >/dev/null
sleep 25
model_log="$(docker compose run --rm dev bash -c \
  "source /etc/capstone_dev.sh && timeout 10 ros2 run capstone_perception perception_node --ros-args -p method:=learned -p learned.model_path:=/nonexistent.onnx -p use_sim_time:=true" 2>&1 || true)"
if printf '%s' "$model_log" | grep -q "classical fallback"; then
  echo "broken_model,broken_model,NA,NA,classical fallback,true,node survived and fell back to the classical method" >> "$csv"
  echo "PASS: classical fallback engaged"
else
  echo "broken_model,broken_model,NA,NA,missing,false,no classical fallback observed" >> "$csv"
  echo "FAIL: no classical fallback observed"
fi

echo "=== frozen clock: pause the simulator while the robot moves ==="
docker rm -f clockfreeze >/dev/null 2>&1 || true
docker compose run -d --name clockfreeze dev bash -c \
  "source /etc/capstone_dev.sh && ros2 launch capstone_integration system.launch.py start_perception:=false episode_id:=clockfreeze" >/dev/null
sleep 12
before="$(docker compose run --rm dev bash -c \
  "source /etc/capstone_dev.sh && timeout 4 ros2 topic echo /cmd_vel --once --field twist.linear.x" 2>/dev/null | tail -n 1)"
docker pause "$sim_container" >/dev/null
sleep 4
during="$(docker compose run --rm dev bash -c \
  "source /etc/capstone_dev.sh && timeout 4 ros2 topic echo /cmd_vel --once --field twist.linear.x" 2>/dev/null | tail -n 1)"
reason="$(docker logs clockfreeze 2>&1 | grep -o 'halt: [A-Z_]*' | tail -n 1 || true)"
docker unpause "$sim_container" >/dev/null
sleep 3
after="$(docker compose run --rm dev bash -c \
  "source /etc/capstone_dev.sh && timeout 4 ros2 topic echo /cmd_vel --once --field twist.linear.x" 2>/dev/null | tail -n 1)"
docker rm -f clockfreeze >/dev/null 2>&1 || true
docker compose stop simulator >/dev/null

recovered=false
if [ -n "$after" ] && awk -v v="$after" 'BEGIN { exit !(v > 0.1) }'; then
  recovered=true
fi
echo "clock_freeze,clock_freeze,NA,NA,$reason,$recovered,cmd_vel before=$before during=$during after=$after" >> "$csv"
echo "clock freeze: before=$before during=$during after=$after reason=$reason"
