# G1 localization (ROS 2 Jazzy / Docker)

Implemented: SE(2) wheel/IMU prediction, point-to-plane scan-to-occupancy-map matching,
NIS gating, Joseph covariance correction, classical covariance and versioned learned
covariance inference with a fixed fallback. No ground-truth input is subscribed.

## Build and test without a simulator

For the course target (Ubuntu 24.04 / Jazzy in Docker), run from the repository
root:

```bash
make test PKG=capstone_localization
bash integration/g1_v1/course_smoke.sh real
```

The following native Humble commands are historical/local diagnostics only,
not the course validation path. Do not reuse Humble build products in Jazzy.

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths ros2_ws/src --packages-select capstone_localization --executor sequential
ROS_DOMAIN_ID=173 colcon test --packages-select capstone_localization --executor sequential
colcon test-result --verbose
source install/setup.bash
ros2 run capstone_localization localization_node --ros-args -p use_sim_time:=true -p method:=classical
```

The working directory for these commands is the repository root. The node uses
`/sensors/wheel_odom`, `/sensors/imu`, `/sensors/scan`, and a transient-local `/map`
whose frame must be `map`. Outputs: `/state/odom` (odom frame),
`/localization/pose` (map frame), `/diagnostics`, and exactly the TF edges
`map -> odom -> base_link`. Disable other publishers of those two TF edges.
The supplied simulation IMU is aligned with base_link; arbitrary rotated IMUs are
not supported yet. LiDAR points are transformed using the timestamped sensor TF.

Parameters: `method` (`classical` or `learned`), `initial_x/y/yaw`,
`wheel_timeout` (0.3 s), `imu_timeout` (0.1 s), `model_path`.
IMU is only used when not in the future and fresh; otherwise wheel yaw is used.
Duplicate/out-of-order wheel data is rejected. Clock rewind over 0.5 s resets the
filter; missing wheel intervals inflate uncertainty but do not invent motion.
Scans must be within 50 ms of the current wheel estimate; older/future/duplicate
scans are rejected rather than incorrectly applied to a different pose. There is
no delayed-state replay or scan deskew yet. Corrections appear at the next wheel tick.

## Learned covariance format

ASCII `G1_MLP_V1` followed by exactly 91 finite floats. An 8-unit tanh hidden layer
stores 7 weights then bias per unit; 3 outputs store 8 weights then bias each.
Inputs, in order: match RMS, inlier fraction, inlier count, geometry ratio,
translation innovation norm, absolute yaw innovation, previous accepted innovation
norm. Outputs use stable softplus and bounds [1e-5,1] m² for x/y and [1e-5,.25]
rad² for yaw. No feature normalization is implicit. Missing/invalid models or
nonfinite inference use fixed covariance, with visible diagnostics.

## Not yet a completed submission

No trained model, held-out A/B benchmark, SLAM-built map/loop-closure artifact,
replay evaluator, calibration plot, or completed report is supplied by this change.
The learned fallback is NOT evidence of a learned-method improvement. Full Webots
integration remains untested by this implementation pass. Sensor plugin and world
changes from earlier work are preserved, not independently validated here.
Do not launch Webots while URRTS or another project is using it: the laptop must
have at most one running simulation. Humble compilation needs no Jazzy rewrite.
