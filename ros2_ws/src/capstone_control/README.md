# `capstone_control` — G4: локальное управление

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G4. Задание: [projects/04_control.md](../../../projects/04_control.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `control` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | action server `/control/follow_path` (`nav2_msgs/action/FollowPath`), `/state/odom`, `/perception/obstacles`, `/sensors/scan`, TF (Transform library — библиотека преобразований между системами координат) |
| Выходы | `/control/cmd_vel_raw` (`geometry_msgs/msg/TwistStamped`), `/diagnostics` |

## Структура

- `include/capstone_control/control_node.hpp`, `src/control_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_control control_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_control` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_control
make test PKG=capstone_control
make shell
```

В `make shell`: `ros2 run capstone_control control_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
