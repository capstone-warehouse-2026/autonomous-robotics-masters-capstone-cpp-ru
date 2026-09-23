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
