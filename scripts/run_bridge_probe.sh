#!/usr/bin/env bash
# Отдельный физический тест драйвера: не запускать Nav2/safety в этом ROS_DOMAIN_ID.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output=${1:-"$root/results/g5-bridge-probe-$(date +%Y%m%d-%H%M%S)"}
domain=${2:-186}
[[ "$domain" =~ ^[0-9]+$ ]] && ((domain <= 232)) || { echo 'ROS_DOMAIN_ID: ожидается 0..232'; exit 2; }
[[ ! -e "$output" ]] || { echo "Каталог уже существует: $output"; exit 2; }
mkdir -p "$output"
output=$(cd "$output" && pwd)
source /opt/ros/jazzy/setup.bash
install_dir=${G5_INSTALL_DIR:-"$root/ros2_ws/install"}
[[ -f "$install_dir/setup.bash" ]] || { echo 'Сначала make g5-system-build'; exit 1; }
source "$install_dir/setup.bash"
package_prefix=$(ros2 pkg prefix capstone_webots_bridge)
bridge="$package_prefix/lib/capstone_webots_bridge/webots_bridge"
probe="$package_prefix/lib/capstone_webots_bridge/bridge_probe"
export WEBOTS_HOME=${WEBOTS_HOME:-"$root/.cache/webots"}
export PATH="$root/.cache/native-deps/usr/bin:$PATH"
export ROS_DOMAIN_ID=$domain
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$output/roslogs"
export WEBOTS_ORIGINAL_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export WEBOTS_TMPDIR=/tmp
export QT_QPA_PLATFORM=xcb
scratch=$(mktemp -d /tmp/g5-bridge-probe.XXXXXX)
export XDG_CONFIG_HOME="$scratch/config"
export XDG_CACHE_HOME="$scratch/cache"
export XDG_RUNTIME_DIR="$scratch/runtime"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
port=$((12400 + domain))
webots_task_pid=
bridge_task_pid=
cleanup() {
  [[ -z "$bridge_task_pid" ]] || kill -- -"$bridge_task_pid" 2>/dev/null || true
  [[ -z "$webots_task_pid" ]] || kill -- -"$webots_task_pid" 2>/dev/null || true
  [[ -z "$bridge_task_pid" ]] || wait "$bridge_task_pid" 2>/dev/null || true
  [[ -z "$webots_task_pid" ]] || wait "$webots_task_pid" 2>/dev/null || true
  rm -rf "$scratch"
}
trap cleanup EXIT
setsid env LD_LIBRARY_PATH="$WEBOTS_HOME/lib/webots:${LD_LIBRARY_PATH:-}" \
  xvfb-run -a -s '-screen 0 1024x768x24' "$WEBOTS_HOME/bin/webots-bin" \
  --batch --mode=realtime --stdout --stderr --no-rendering --port="$port" \
  "$root/simulation/integration/warehouse.wbt" > "$output/webots.log" 2>&1 &
webots_task_pid=$!
setsid "$WEBOTS_HOME/webots-controller" --robot-name=g5_robot --port="$port" \
  "$bridge" > "$output/bridge.log" 2>&1 &
bridge_task_pid=$!
set +e
"$probe" > "$output/probe.log" 2>&1
result=$?
set -e
if rg -q '^ERROR:' "$output/webots.log"; then result=1; fi
python3 - "$root" "$output" "$bridge" "$probe" "$result" <<'PY'
import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

root, output, bridge, probe = map(Path, sys.argv[1:5])
text = (output / 'probe.log').read_text()
metrics = dict(re.findall(r'(physical_distance|encoder_distance|physical_v|encoder_v|yaw_rate|contact)=([-0-9.]+)', text))
files = [root / 'simulation/integration/warehouse.wbt',
         root / 'simulation/integration/map.pgm',
         root / 'ros2_ws/src/capstone_webots_bridge/src/webots_bridge.cpp',
         root / 'ros2_ws/src/capstone_webots_bridge/include/capstone_webots_bridge/odometry.hpp',
         root / 'ros2_ws/src/capstone_webots_bridge/test/bridge_probe.cpp', bridge, probe]
audit = {
    'schema_version': 1,
    'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'scope': 'physical Webots bridge only; not autonomous navigation',
    'webots_version': 'R2025a',
    'git_commit': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
    'git_dirty': bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True)),
    'passed': int(sys.argv[5]) == 0 and 'PASS:' in text,
    'exit_code': int(sys.argv[5]),
    'metrics': {key: float(value) for key, value in metrics.items()},
    'checks': ['real LiDAR: front=2.5±0.12m, back=2.9±0.12m',
               'physical displacement 0.5..1.2m and encoder disagreement <=0.12m',
               'no obstacle contacts', 'physical and encoder speed after watchdog <0.03m/s',
               'reset acknowledgement after physical step',
               'post-reset map x=-3±0.05m and encoder x=0±0.05m'],
    'sha256': {str(path.relative_to(root)) if path.is_relative_to(root) else str(path):
               hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
}
(output / 'audit.json').write_text(json.dumps(audit, indent=2, ensure_ascii=False) + '\n')
PY
cat "$output/probe.log"
echo "Журнал и аудит: $output"
exit "$result"
