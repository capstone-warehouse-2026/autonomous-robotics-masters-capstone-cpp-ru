# G5 — миссии и диспетчеризация

Реализованы классический EDD-диспетчер и DQN внутри одного дерева BehaviorTree.CPP.
Исполнение миссии, нейросети и расчёт экспериментальных метрик — C++17.
Python/NumPy используется только вне контура робота для обучения и подготовки данных.

## Быстрый запуск без G1–G4

Нужны Docker Engine и Compose v2; первоначальная подготовка — `./scripts/setup_host.sh`.
Из корня репозитория:

```bash
make g5-test
make g5-demo                       # классический метод
make g5-demo G5_METHOD=learned      # DQN из поставляемой модели
make g5-smoke                      # конечный эпизод, manifest и C++ метрики на хосте
```

Демо запускает отдельный `capstone_debug_navigation`: часы, одометрию, локализацию,
планирование прямого пути и кинематическое движение через настоящие ROS actions.
Затем G5 обслуживает две заявки из `config/demo.yaml`. Журнал по умолчанию
`/tmp/g5-debug-events.csv` внутри контейнера; для сохранения на хост передайте
`event_log:=/workspace/results/g5-demo/events.csv` при ручном запуске.
`Ctrl+C` запрашивает отмену принадлежащей G5 цели и ограниченно ожидает результат
и свежую нулевую скорость перед выключением ROS.

Это тест G5 с упрощённой навигацией, не Webots и не доказательство работы G1–G4.
Настоящие пакеты других групп не подменяются: отладочный пакет запускается отдельно.
Демо использует ROS_DOMAIN_ID=75; обычная инфраструктура проекта — 42.

Ручной запуск в `make shell` после сборки:

```bash
export ROS_DOMAIN_ID=75
ros2 launch capstone_debug_navigation demo.launch.py method:=learned \
  event_log:=/workspace/results/g5-demo/events.csv
```

Для native ROS 2 Jazzy (Ubuntu 24.04, все зависимости должны быть установлены):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_ws/src --packages-up-to capstone_debug_navigation \
  capstone_benchmark --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ROS_DOMAIN_ID=75 ros2 launch capstone_debug_navigation demo.launch.py method:=classical
```

`make g5-smoke` сам сбрасывает стенд, запускает миссию и завершается после результата.
Артефакты остаются в `results/g5-ros-episode/`; для второго метода задайте
`make g5-smoke G5_METHOD=learned G5_SMOKE_OUTPUT=/workspace/results/g5-ros-learned`.
Ошибка миссии приводит к ненулевому коду, а не к фиктивному успешному smoke.

## Правила и структура

- `domain.hpp/cpp`: схема сценария, очередь, переходы состояний, единый набор признаков.
- `dispatch.hpp/cpp`: EDD и выполнение обученной сети 32→64 ReLU→5.
- `navigation_client.hpp/cpp`: принятие action, feedback, результат, отмена и остановка.
- `mission_node.hpp/cpp`: дерево, обслуживание, события, готовность и сброс эпизода.
- `behavior_trees/delivery.xml`: `WaitReady → SelectJob → NavigatePickup → PickupService
  → NavigateDropoff → DropoffService → CompleteJob`.
- `test/`: тесты очереди, inference против Python, настоящего ROS action обмена и миссии.

Из очереди выбираются четыре старейшие поступившие заявки по `(release_time, job_id)`.
EDD минимизирует `(due_time, estimated_route_length, job_id)` внутри этих четырёх.
DQN выбирает слот 0–3 либо ожидание на одну секунду времени симуляции; пустые слоты
маскируются. Остальные заявки сохраняются. Новые заявки не прерывают активную доставку.
Отсутствующая/повреждённая модель или некорректный inference приводят к EDD fallback
с диагностикой и записью события. Классический метод не загружает модель.

На этап навигации разрешены **две попытки всего**. Перед повтором нужно получить
конечный результат старой цели и свежую одометрию с нулевой скоростью. Подтверждение
запроса отмены ещё не считается остановкой. При неизвестном результате/движении
повтор запрещён. Отказ после pickup оставляет миссию в fault: нельзя исчезнуть с грузом
и выбрать следующую заявку. Исправление и сброс сценария выполняются явно.

Внешняя замена задания выполняется через отмену текущей миссии, ожидание конечного
статуса/остановки и сброс; DQN самостоятельно не прерывает доставку.

## Сценарии и параметры

`manifest_path` принимает YAML или JSON; пример — `config/demo.yaml`.
Обязательные поля: `episode_id`, `initial_station`, `stations`, `jobs`.
Станция: `id,x,y,yaw`; заявка: `id,pickup,dropoff,release_time,due_time,
pickup_service,dropoff_service`. Все времена в секундах относительно начала эпизода,
координаты станций — `map`, метры/радианы. `horizon` по умолчанию 300 с.
Необязательные таблицы `travel_times`, `congestion`, `blocked` нужны для изолированных
экспериментов, не предоставляют доступ к будущим arrival events.

`config/mission.yaml` содержит таймауты и режим готовности. Для реальной интеграции
использовать `require_safety=true`: нужны свежие локализация, одометрия, скан,
карта препятствий, heartbeat safety monitor и оба action-сервера.
Ожидание action/остановки и зависание часов контролируются монотонным временем.
Service/wait и расписание заявок используют время симуляции.

Интерфейсы относительные, поэтому их можно изолировать namespace:

| Интерфейс | Назначение |
|---|---|
| `planning/compute_path` | клиент `nav2_msgs/action/ComputePathToPose` |
| `control/follow_path` | клиент `nav2_msgs/action/FollowPath` |
| `state/odom`, `localization/pose` | остановка и положение для диспетчеризации |
| `diagnostics`, `sensors/scan`, `perception/obstacles` | готовность и возраст входов |
| `mission/status` | состояние, счётчики и причины через DiagnosticArray |
| `mission/start`, `mission/cancel`, `mission/reset` | сервисы Trigger |

`mission/reset` сбрасывает G5; он не перемещает робота и не перезапускает чужие узлы.
Общий runner должен сначала безопасно закончить старый эпизод и сбросить симулятор.
При сбросе часов G5 отменяет старую цель и меняет episode ID; старые callbacks
не имеют права продвинуть новую миссию. Только safety monitor публикует `/cmd_vel`.

## Обучение и оценка

```bash
make g5-train                     # 3 seed + 3 seed ablation, CPU, 1000 эпизодов на seed
make g5-model                     # экспорт весов, сценариев и эталонов
make g5-eval                      # C++: 100 test-сценариев × 7 методов = 700 эпизодов
```

Модели уже находятся в `models/`; переобучение для запуска демо не требуется.
Формат — открытый JSON с весами float32 и версией признаков, а не ONNX.
C++ inference не требует Python, ONNX Runtime или GPU. Контракт и воспроизводимость:
[ml/mission/README.md](../../../ml/mission/README.md).

`make g5-eval` пишет `results/g5-isolated/metrics.csv`, `events.csv`, `manifest.yaml`
и `summary.csv`; `summary_all_metrics.csv` содержит интервалы остальных метрик,
число валидных наблюдений и число пар, без замены `NA` на ноль.
Повторный запуск в тот же каталог отклоняется, чтобы не потерять
результаты; задайте `G5_RESULTS=/workspace/results/g5-next`.
Seed обучения показаны отдельно; ablation действительно переобучен без congestion.
Отказы и незавершённые заявки не исключаются из знаменателя on-time rate.

Полный перенос в Webots требует готовых action-серверов G3/G4, состояния и датчиков G1.
Профили C0–C6 подготовлены в `capstone_bringup`, но сами по себе не реализуют чужие
алгоритмы. Текущие экспериментальные выводы относятся к изолированной диспетчеризации
и отладочной ROS-симуляции. См. [интеграционные границы](../../../docs/G5_INTEGRATION.md).
