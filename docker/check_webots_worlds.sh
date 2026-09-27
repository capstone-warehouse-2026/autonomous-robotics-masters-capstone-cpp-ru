#!/usr/bin/env bash
# Loads every world of simulation/worlds in Webots (virtual display, window rendered) and fails on any ERROR line
# or when the scene does not finish loading (Webots then waits for the <extern> robot controller).
#   check_webots_worlds.sh [--settle SECONDS] [WORLD.wbt ...]
# --settle keeps the loaded world open a little longer (used by scripts/update_webots_assets.sh).
set -euo pipefail

settle=0
if [ "${1:-}" = "--settle" ]; then
  settle="$2"
  shift 2
fi
worlds=("$@")
if [ "${#worlds[@]}" -eq 0 ]; then
  worlds=(/opt/capstone/simulation/worlds/*.wbt)
fi

status=0
for world in "${worlds[@]}"; do
  log="$(mktemp)"
  # Own session, so the whole xvfb-run/Xvfb/Webots group can be stopped together.
  setsid xvfb-run --auto-servernum "$WEBOTS_HOME/webots" --batch --mode=fast --stdout --stderr \
    "$world" > "$log" 2>&1 &
  pid=$!
  for _ in $(seq 1 120); do
    if grep -q "extern controller: Waiting" "$log" || grep -q "^ERROR:" "$log" || ! kill -0 "$pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  sleep "$settle"
  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  if grep -q "^ERROR:" "$log" || ! grep -q "extern controller: Waiting" "$log"; then
    grep -E "^(ERROR|WARNING):" "$log" | sort | uniq | head -n 20 || true
    echo "FAIL: $(basename "$world") does not load cleanly without network; if new standard objects were added, run scripts/update_webots_assets.sh" >&2
    status=1
  else
    echo "OK: $(basename "$world") loads without network"
  fi
  rm -f "$log"
done
exit "$status"
