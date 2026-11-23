# `capstone_planning` — G3: глобальное планирование

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Пакет группы G3. Задание: [projects/03_planning.md](../../../projects/03_planning.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md), описание и результаты метода A: [docs/G3_METHOD_A.md](../../../docs/G3_METHOD_A.md).

Состояние: метод A (classical) — A* (A-star — алгоритм поиска пути с оценкой уже пройденной стоимости и эвристикой оставшегося пути) с фиксированной стоимостью длины, clearance и наблюдаемой occupancy — реализован и подключён к action server. Метод B (learned) не реализован: `method: learned` использует стоимость метода A как fallback и сообщает об этом в `/diagnostics`.

| | |
|---|---|
| Входы | `/map`, `/perception/obstacles` (G2, неинфлированная сетка, обычно в `odom`), `/localization/pose`, TF (Transform library — библиотека преобразований между системами координат) `map ← odom` |
| Выходы | action server `/planning/compute_path` (`nav2_msgs/action/ComputePathToPose`), `/diagnostics` |

## Структура

- Библиотека `capstone_planning_algorithms` (без ROS (Robot Operating System — программная платформа для робототехники)):
  - `grid.hpp` — сетка в формате `nav_msgs/OccupancyGrid`, 2D-преобразования;
  - `map_fusion.hpp` — объединение `/map` и локальной сетки G2 в `map`;
  - `cost_map.hpp` — inflation (hard collision mask: радиус робота + запас), clearance и стоимость метра;
  - `grid_search.hpp` — A* / Dijkstra на 8-связной сетке без corner cutting, cancel, deadline, явные статусы отказа;
  - `planner.hpp` — путь в виде поз, длина, clearance, проверка блокировки пути после обновления карты;
  - `scenarios.hpp` — синтетические карты сценариев задания (узкий проход, перекрытый коридор, два маршрута, недостижимая цель, склад).
- `include/capstone_planning/planning_node.hpp`, `src/planning_node.cpp` — узел: подписки, fusion, action server с отдельным потоком поиска, preemption/cancel, метрики в `/diagnostics`.
- `src/main.cpp` — `ros2 run capstone_planning planning_node`; `src/planning_benchmark.cpp` — офлайн-бенчмарк сценариев.
- `config/planning.yaml`, `launch/planning.launch.py` — параметры и запуск.
- `test/test_algorithms.cpp` — тесты алгоритмов (в том числе A* против Dijkstra на случайных картах), `test/test_planning_node.cpp` — тесты узла и action.

## Поведение action

| Ситуация | Результат |
|---|---|
| Путь найден | `SUCCEEDED`, `error_code = NONE`, `nav_msgs/Path` в `map`: точный старт, центры клеток, точная цель с её yaw |
| Цель в препятствии или в маске робота | `ABORTED`, `GOAL_OCCUPIED` |
| Старт в препятствии; старт в маске без выхода за 0.5 м | `ABORTED`, `START_OCCUPIED` |
| Цель недостижима | `ABORTED`, `NO_VALID_PATH` |
| Старт/цель вне карты | `ABORTED`, `START_OUTSIDE_MAP` / `GOAL_OUTSIDE_MAP` |
| Поиск дольше `planning_timeout` (200 мс) | `ABORTED`, `TIMEOUT` |
| Нет TF, нет свежей `/localization/pose` при `use_start: false`, NaN в позе | `ABORTED`, `TF_ERROR` |
| Нет `/map`, нет свежей `/perception/obstacles` при `require_obstacles: true` | `ABORTED`, `UNKNOWN` с причиной в `error_msg` |
| Cancel | `CANCELED` |
| Новая цель во время поиска | старая `ABORTED` с «preempted by a newer goal», выполняется новая |

`planner_id` игнорируется. Если последний выданный путь стал непроходимым после обновления `/perception/obstacles`, узел сразу публикует `/diagnostics` с уровнем WARN; новый путь запрашивает G5.

## Команды

```bash
make build PKG=capstone_planning
make test PKG=capstone_planning
make shell
```

В `make shell`:

```bash
ros2 launch capstone_planning planning.launch.py method:=classical
ros2 run capstone_planning planning_benchmark 100
ros2 action send_goal /planning/compute_path nav2_msgs/action/ComputePathToPose \
  "{use_start: true, start: {header: {frame_id: map}, pose: {position: {x: -8.0, y: 0.0}, orientation: {w: 1.0}}}, \
    goal: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 6.6}, orientation: {w: 1.0}}}}"
```

Пока G2 не публикует `/perception/obstacles`, запускайте с `-p require_obstacles:=false`: тогда планирование идёт только по `/map`.

## Зависимости

Библиотеки добавляются строкой `<depend>ключ</depend>` в `package.xml` и `find_package(...)` в `CMakeLists.txt`. Ключ — это имя пакета ROS, например `sensor_msgs` или `cv_bridge`, либо ключ [rosdep](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml), например `libopencv-dev` или `eigen`. Следующий `make build` сам пересоберёт образ разработки с новой зависимостью. Если библиотеки нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), создайте issue с меткой `infra`.
