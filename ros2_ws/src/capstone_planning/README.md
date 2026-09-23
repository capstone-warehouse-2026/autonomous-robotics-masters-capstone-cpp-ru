# `capstone_planning` — G3: глобальное планирование

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G3. Задание: [projects/03_planning.md](../../../projects/03_planning.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `planning` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | `/map`, `/perception/obstacles`, `/localization/pose` |
| Выходы | action server `/planning/compute_path` (`nav2_msgs/action/ComputePathToPose`), `/diagnostics` |

## Структура

- `include/capstone_planning/planning_node.hpp`, `src/planning_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_planning planning_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_planning` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_planning
make test PKG=capstone_planning
make shell
```

В `make shell`: `ros2 run capstone_planning planning_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
