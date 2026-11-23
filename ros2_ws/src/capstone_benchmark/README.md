# `capstone_benchmark` — G5: C++ benchmark runner и расчёт метрик

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G5. Задание: [projects/05_mission.md](../../../projects/05_mission.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `benchmark_runner` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | scenario manifest, `/evaluation/ground_truth` (только evaluator), `/mission/status` |
| Выходы | CSV (Comma-Separated Values — табличный текстовый формат со значениями, разделёнными запятыми) результатов и run manifest по [протоколу экспериментов](../../../docs/EXPERIMENTS.md) |

## Структура

- `include/capstone_benchmark/benchmark_runner_node.hpp`, `src/benchmark_runner_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_benchmark benchmark_runner_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_benchmark` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_benchmark
make test PKG=capstone_benchmark
make shell
```

В `make shell`: `ros2 run capstone_benchmark benchmark_runner_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.

### G1 offline evaluation of one episode (C++)

After colcon build --packages-select capstone_benchmark in Jazzy Docker:

~~~bash
ros2 run capstone_benchmark g1_trajectory_eval INPUT.csv METRICS.json PAIRED.csv START_NS
~~~

Input header: stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad. Each of truth (independent ground truth) and map (G1 global estimate) must be explicitly in the same map frame; optional wheel and fused streams are in odom. One CSV = one episode. Provide finite planar poses, increasing unique integer simulation timestamps in nanoseconds per stream, and explicit evaluation start. Existing live smoke raw CSV validates frames in its recorder but omits the frame column; use --legacy-smoke explicitly only for that recording path. Future captures should contain frame_id.

Primary ATE/p95/yaw use common-map coordinates with no spatial fit. Match at estimate timestamps via exact or bracketed independent truth, linear XY and shortest-arc yaw; no extrapolation or time shift. Maximum bracket 40 ms (50 Hz GT, permits one missing sample). Per-stream exact/interpolated/outside/gap counts and paired poses are saved. p95 is nearest rank. 1 s RPE uses relative SE(2) increments evaluated at exactly t+1 s, with bounded interpolation on paired trajectories; may differ from the prior diagnostic's next-sample convention. Loss is >0.5 m or >15 degrees continuously observed for strictly >1 s; gaps >40 ms break continuity, and a loss cut short by a gap or end is right-censored. Odometry and first-pose-aligned baselines are supplemental.

To verify on Apparat with the already cached Jazzy image, no network and no simulator:

~~~bash
bash ~/Baumanka/Mobile_robots/HW1-official-downloads/test-hw1-cpp-evaluator.sh
~~~

The wrapper builds and tests in Docker, reprocesses the existing 17.864 s-start trace, preserves and hashes the original files. This evaluator is not the full benchmark collector/runner, aggregate report, NEES/NIS or method A/B.

### Offline G1 episode batch (C++)

In the Jazzy image: ros2 run capstone_benchmark g1_batch_eval MATRIX.csv NEW_OUTPUT_DIR ANALYSIS_SEED

Matrix is a simple unquoted CSV manifest, one planned row per (episode, method):

~~~text
episode_id,method,scenario_id,layout_id,scenario_seed,training_seed,evidence,input_id,terminal,start_ns,format,trace_sha256,trace_path
~~~

Methods A/B denote saved estimator outputs; use fixture only for synthetic validation. Evidence is smoke, fixture or test; no grouping across these strata. input_id is the SHA-256 of the replay input (sensor bag/file bundle manifest); trace_sha256 is the SHA-256 of the CSV being scored; verify actual byte hashes in the launcher. Pure synthetic fixtures and the legacy smoke may use a trace hash as a placeholder input identifier, not as evidence of shared sensor replay. Terminal is ready, missing, timeout or crash. A ready row needs a relative path to the trajectory and framed or legacy_smoke input; formal test rows require frame IDs and training seed for B. Use na for non-trained runs. One fixed training seed per B matrix; analyze additional training seeds separately. Predeclare episode IDs, scenario/layout and seeds. Indicate absent expected methods as planned missing rows where possible. Scope is processing previously saved trajectories; this does not run either estimator or launch simulation.

Outputs: copied matrix, batch manifest, episodes.csv (failures have blank metrics), aggregates.csv (episode-level mean, median, sample SD and eligible bootstrap mean CI), paired.csv (test-only A/B pair ledger + mean difference/CI). Pairing checks same episode, scenario, layout, scenario seed, input ID, start time and identical independent ground-truth stream. Repeated inputs are not independent. Test CIs require completed groups with at least 20 samples and two layouts; paired CIs use 10,000 fixed-seed bootstrap resamples over paired episode differences. These eligibility guards are necessary but not sufficient for official acceptance (disjoint data splits, predeclared coverage, fault cases, model hashes and full experiment manifest still required). Single-group smoke and synthetic fixture never produce CIs. Exit 2 means some planned rows unscored; exit 1 means malformed input or output error.

On Apparat run bash ~/Baumanka/Mobile_robots/HW1-official-downloads/test-hw1-cpp-batch.sh using the cached Jazzy image. It builds + tests with networking disabled, then scores one real nominal smoke and two explicitly numerical fixture rows plus a declared missing fixture; no real method B is simulated or claimed.
