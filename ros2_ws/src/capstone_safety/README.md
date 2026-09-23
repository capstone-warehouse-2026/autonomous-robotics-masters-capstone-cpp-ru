# `capstone_safety` — G4: независимый защитный узел

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G4. Задание: [projects/04_control.md](../../../projects/04_control.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `safety_monitor` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | `/control/cmd_vel_raw`, `/sensors/scan`, `/state/odom` |
| Выходы | `/cmd_vel` (`geometry_msgs/msg/TwistStamped`), `/diagnostics` |

## Структура

- `include/capstone_safety/safety_monitor_node.hpp`, `src/safety_monitor_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_safety safety_monitor_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_safety` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_safety
make test PKG=capstone_safety
make shell
```

В `make shell`: `ros2 run capstone_safety safety_monitor_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
