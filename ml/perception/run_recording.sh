#!/usr/bin/env bash
# Runs inside the simulator container (`make g2-dataset`): makes a recording copy of the world,
# starts the simulator without a window in fast mode and record_dataset.py as its extern supervisor.
# Arguments are passed to record_dataset.py; RUN selects the output directory. LAYOUT (an ID from
# splits.json) records a generated warehouse with its own pose seed; without it WORLD (a file of
# simulation/worlds, default warehouse.wbt) is recorded with the seed given in the arguments.
set -euo pipefail
cd "$(dirname "$0")"

world=/tmp/g2_dataset_world.wbt
layout_args=()
if [ -n "${LAYOUT:-}" ]; then
  source_world=/tmp/g2_layout.wbt
  info="$(python3 generate_layout.py --layout "$LAYOUT" --template /opt/capstone/simulation/worlds/warehouse.wbt \
    --protos-dir /opt/capstone/simulation/protos --out "$source_world")"
  seed="$(python3 -c 'import json, sys; print(json.loads(sys.argv[1])["seed"])' "$info")"
  world_name="generated:$LAYOUT"
  layout_args=(--layout-id "$LAYOUT" --seed "$seed")
else
  world_name="${WORLD:-warehouse.wbt}"
  source_world="/opt/capstone/simulation/worlds/$world_name"
fi

# DEF names for the supervisor, absolute paths for own PROTOs (the copy lives outside worlds/).
sed -e 's|^TiagoBase {|DEF DATASET_ROBOT TiagoBase {|' \
    -e 's|^TexturedBackground {|DEF DATASET_BACKGROUND TexturedBackground {|' \
    -e 's|^TexturedBackgroundLight {|DEF DATASET_LIGHT TexturedBackgroundLight {|' \
    -e 's|^RectangleArena {|DEF DATASET_ARENA RectangleArena {|' \
    -e 's|"\.\./protos/|"/opt/capstone/simulation/protos/|g' \
    "$source_world" > "$world"
cat >> "$world" <<'WBT'
Robot {
  name "dataset_recorder"
  controller "<extern>"
  supervisor TRUE
}
WBT
for name in DATASET_ROBOT DATASET_BACKGROUND DATASET_LIGHT DATASET_ARENA; do
  grep -q "^DEF $name " "$world" || { echo "$world_name: no node for $name" >&2; exit 1; }
done

SIM_WORLD="$world" SIM_MODE=fast bash /opt/capstone/start_simulator.sh > /tmp/simulator.log 2>&1 &
simulator=$!
out="/data/${RUN:-run}"
mkdir -p "$out"
# The simulator log (Webots, robot driver, camera) is kept next to the frames for diagnosis.
trap 'kill "$simulator" 2>/dev/null || true; wait "$simulator" 2>/dev/null || true; cp /tmp/simulator.log "$out/simulator.log" 2>/dev/null || true' EXIT

# colcon setup scripts read unset variables, so -u is off while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/capstone/sim_ws/install/setup.bash
set -u
"$WEBOTS_HOME/webots-controller" --robot-name=dataset_recorder record_dataset.py \
  --out "$out" --world "$world_name" "$@" "${layout_args[@]}"
