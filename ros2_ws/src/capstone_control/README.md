# `capstone_control` — G4: локальное управление

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Пакет группы G4. Задание: [projects/04_control.md](../../../projects/04_control.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md), поведение `FollowPath`: [ADR 0004](../../../docs/adr/0004-g4-follow-path.md).

Состояние: готовы алгоритмы следования пути без ROS (Robot Operating System — программная платформа для робототехники) и их тесты. Узел `control` пока только проверяет параметр `method` (`classical` или `learned`) и ничего не публикует; action server — следующий шаг.

| | |
|---|---|
| Входы | action server `/control/follow_path` (`nav2_msgs/action/FollowPath`), `/state/odom`, `/perception/obstacles`, `/sensors/scan`, TF (Transform library — библиотека преобразований между системами координат) |
| Выходы | `/control/cmd_vel_raw` (`geometry_msgs/msg/TwistStamped`), `/diagnostics` |

## Структура

- Библиотека `capstone_control_algorithms` (без ROS):
  - `geometry.hpp` — `Pose2D`, `Command`, `Path`, нормализация угла;
  - `path_tracker.hpp` — проверка пути и проекция робота на путь (пройдено, осталось, поперечная ошибка);
  - `controller.hpp` — общий интерфейс контроллеров: одинаковый `ControlInput` для pure pursuit, MPC (Model Predictive Control — управление с прогнозирующей моделью) и PPO (Proximal Policy Optimization — алгоритм оптимизации политики с ограничением величины её обновления);
  - `pure_pursuit_controller.hpp` — временный классический контроллер до готовности MPC;
  - `command_limiter.hpp` — ограничение скоростей и ускорений, общее для всех контроллеров;
  - `checkers.hpp` — достижение цели и обнаружение застревания.
- `include/capstone_control/control_node.hpp`, `src/control_node.cpp` — класс узла; код собирается в библиотеку, общую для исполняемого файла и тестов.
- `src/main.cpp` — запуск узла `ros2 run capstone_control control_node`.
- `test/` — тесты gtest, запускаются `make test PKG=capstone_control` и в CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки). `test_closed_loop.cpp` прогоняет идеальную модель дифференциального робота с контроллером по прямой, дуге, углу 90°, с разворотом и с отклонением скорости ±20%.

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
