# `capstone_mission` — G5: диспетчеризация заявок и дерево поведения

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G5. Задание: [projects/05_mission.md](../../../projects/05_mission.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `mission` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | manifest заявок, action clients `/planning/compute_path` и `/control/follow_path` |
| Выходы | `/mission/status` (`diagnostic_msgs/msg/DiagnosticArray`), журнал событий заявок |

## Структура

- `include/capstone_mission/mission_node.hpp`, `src/mission_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_mission mission_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_mission` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_mission
make test PKG=capstone_mission
make shell
```

В `make shell`: `ros2 run capstone_mission mission_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
