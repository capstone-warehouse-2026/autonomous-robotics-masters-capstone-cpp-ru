# `capstone_localization` — G1: локализация и геометрическая карта

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Заготовка пакета группы G1. Задание: [projects/01_localization.md](../../../projects/01_localization.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md). Сейчас узел `localization` только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует.

| | |
|---|---|
| Входы | `/sensors/wheel_odom`, `/sensors/imu`, `/sensors/scan`, `/map` |
| Выходы | `/state/odom`, `/localization/pose`, TF (Transform library — библиотека преобразований между системами координат) `map → odom → base_link`, `/diagnostics` |

## Структура

- `include/capstone_localization/localization_node.hpp`, `src/localization_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_localization localization_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_localization` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки).

Новые исходные файлы и тесты добавляются в `CMakeLists.txt` рядом с существующими.

## Команды

```bash
make build PKG=capstone_localization
make test PKG=capstone_localization
make shell
```

В `make shell`: `ros2 run capstone_localization localization_node --ros-args -p method:=learned`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS (Robot Operating System — программная платформа для робототехники), например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
