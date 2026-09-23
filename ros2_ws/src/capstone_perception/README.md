# `capstone_perception` — G2: восприятие препятствий и локальная карта

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G2. Задание: [projects/02_perception.md](../../../projects/02_perception.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `perception` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | `/sensors/rgb`, `/sensors/depth`, `/sensors/camera_info`, TF (Transform library — библиотека преобразований между системами координат) |
| Выходы | `/perception/obstacles` (`nav_msgs/msg/OccupancyGrid`, frame `odom`), `/diagnostics` |

## Структура

- `include/capstone_perception/perception_node.hpp`, `src/perception_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_perception perception_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_perception` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_perception
make test PKG=capstone_perception
make shell
```

В `make shell`: `ros2 run capstone_perception perception_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
