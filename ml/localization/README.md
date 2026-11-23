# G1 reproducible data → training → held-out A/B

**Course target: Ubuntu 24.04 / ROS 2 Jazzy in Docker.** The Humble commands
below document the historical local workflow, not the reference build or final
verification path. Start with the [G1 integration fixture](../../integration/g1_v1/README.md),
then see [train_0 mapping](train0_commissioning.md) and the [classical replay diagnostic](train0_scoring_readout.md).

Status: tooling, not a completed experiment. No real bag, SLAM map or trained
submission model was present in this checkout when inspected on 2026-11-13.
Synthetic fixtures in `test_pipeline.py` are unit tests, NOT empirical evidence.
No Webots instance is started by any script in this directory.

## Resource and ownership rule

URRTS owns the Webots slot until the coordinator explicitly transfers it. Never
start a second world, including headless. Build sequentially (`MAKEFLAGS=-j1`).
Use a separate ROS_DOMAIN_ID, for example 173, and ROS_LOCALHOST_ONLY=1.

## 1. Mapping pass (after obtaining the simulation slot)

Build `capstone_sim`, `capstone_sim_rgbd`, and `capstone_localization` on Humble
using the installed Webots overlay; the current robot URDF includes the RGB-D
plugin too. Start the existing `simulation/launch/warehouse.launch.py` only after
checking the Webots slot. Do not make another launch while it is running.

Start wheel/IMU odometry and SLAM in separate terminals:

```bash
ros2 run capstone_localization localization_node --ros-args \
  -p use_sim_time:=true -p publish_map_tf:=false
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ml/localization/slam_humble.yaml
```

During mapping, only SLAM publishes map→odom; G1 publishes odom→base_link. Drive
a safe closed route through the map, revisit the start, retain the pose graph and
before/after loop-closure evidence. Save the map with the installed Nav2 map saver:

```bash
ros2 run nav2_map_server map_saver_cli -f /absolute/path/to/frozen_map
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/absolute/path/to/pose_graph'}"
```

This procedure is prepared, NOT yet simulation-validated. Inspect SLAM diagnostics
and map artifacts before accepting them. Save world hash, spawn pose, sensor
settings, mapping route, duration, and loop-closure observations beside the map.

## 2. Frozen-map recording passes

Stop SLAM before localization comparison. Serve its saved map using
`nav2_map_server map_server`, configure and activate its lifecycle. Do not record
an evolving /map. Map and GT must share a declared coordinate system: the current
simulator GT publishes world coordinates while calling its frame `map`. If SLAM
starts at the robot spawn, derive a fixed world→map transform from that known
spawn BEFORE evaluation. Do not fit or align either trajectory to GT afterward.

Choose distinct worlds AND trajectories for train, validation, test. Include
normal, slip, corridor and IMU/LiDAR dropout cases. Changing just a seed or renaming
the same world is not an independent world. Preserve fault windows and settings.
Record with an already-running simulator and frozen-map publisher:

```bash
bash ml/localization/record.sh /absolute/path/bags/train_01 180
python3 ml/localization/export_bag.py /absolute/path/bags/train_01 data/g1/train_01 \
  --world-file simulation/worlds/warehouse.wbt --world train_world_01 --trajectory train_route_01 --scenario normal --split train \
  --initial 0 0 0 --gt-to-map 8 0 0
```

The numbers above illustrate a map whose origin is a known world spawn (-8,0,0);
replace them with the actual predeclared transform and map-frame starting pose.
Use a ROS-enabled Python containing rosbag2_py, rclpy and rosidl_runtime_py.
Export rejects absent/changing map, unsupported extrinsics, reset-spanning bags,
missing sensor/GT streams and invalid frames. Files are hash-locked in manifest.
GT stays in a separate CSV; the estimator executable never receives that file.
Memory use currently scales with bag size, so keep recordings bounded.

## 3. Build replay, fit, freeze, evaluate

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths ros2_ws/src --packages-select capstone_localization --executor sequential
source install/setup.bash
REPLAY="$PWD/install/capstone_localization/lib/capstone_localization/localization_replay"
python3 ml/localization/pipeline.py train --replay "$REPLAY" --output artifacts/g1/model \
  data/g1/train_01 data/g1/validation_01
python3 ml/localization/pipeline.py evaluate --replay "$REPLAY" \
  --model-dir artifacts/g1/model --output artifacts/g1/test data/g1/test_01
```

Training uses raw scan-match residual-to-GT squared errors as covariance labels,
including gated match candidates, not only accepted corrections. Network: 7→8
tanh→3 stable softplus with identical covariance limits to the classical path.
Gaussian diagonal NLL trains on train; minimum validation NLL chooses the epoch.
Feature normalization is fitted on train only and absorbed into exported weights.
A scalar fixed-covariance multiplier for A is calibrated on validation residuals,
then frozen. B uses that SAME frozen covariance for fallback. Both methods share
the exact C++ matcher/filter/gate, input events, initial pose, map and bounds.
No-history ablation: repeat training with `--no-history` and evaluate separately.

Exports: `model.txt`, training metadata/model hash, prediction CSVs, `metrics.json`,
SVG trajectories and `REPORT.md`. Reports compute ATE/RPE/p95, NEES/NIS, qualified
loss intervals/recovery, scan acceptance, fallback and kernel timing. Latency is
CPU replay kernel timing, NOT end-to-end ROS callback/scheduling latency; the
20 ms overrun fraction must not be called a measured ROS deadline-miss rate.
Map IoU/build duration is separate:

```bash
python3 ml/localization/map_metrics.py frozen-map.txt independently-known-map.txt --build-seconds 180
```

Only matching, predeclared map lattices are accepted. An independently known
reference map is required. No GT occupancy map is invented from SLAM output.

## Checks without Webots

```bash
G1_REPLAY="$REPLAY" python3 -m unittest discover -s ml/localization -v
ROS_DOMAIN_ID=173 colcon test --packages-select capstone_localization --executor sequential
colcon test-result --verbose
```

Known remaining acceptance work: validate real sensor/world runtime, complete
map/loop-closure evidence, collect independent data, run frozen A/B and ablation,
inspect plots and failure episodes, measure ROS end-to-end timing, share actual
bags/TF contract with G2/G3/G4, and complete the defense/report. A trained model
from the unit tests must never be presented as the final learned method.

An evaluator-only GT sidecar can be decoded without being republished in ROS.
Pass --ground-truth-bag PATH when the input and truth were recorded separately;
the input must not also contain ground truth, and the sidecar must contain only
/evaluation/ground_truth in map → base_link. The bags must overlap in simulation
time. The exporter hashes each bag file separately. The legacy single-bag form
still works. Ground truth is never included in events.txt.

For the nominal Apparat commissioning capture, always use --split pilot with
the two sidecar folders and the recorded, declared initial pose and world→map
transform. Outputs are marked recorded_pilot and are not eligible for model
training, validation or scoring. The exporter also rejects other split values
when the source path or adjacent provenance identifies this HW1 pilot. Keep
provenance with any copied bag; this guard is not a substitute for maintaining
the experiment's disjoint, predeclared splits.
