# Контракты интеграции v1

Сокращения раскрыты при первом употреблении; [полный словарь терминов](GLOSSARY.md).

Это спецификация для реализации студентами. Стартовый код реализует только clock probe. До конца недели 2 группы фиксируют контракт и записывают любые изменения в ADR (Architecture Decision Record — запись об архитектурном решении)/PR (Pull Request — запрос на включение изменений в репозиторий), согласованный с обеими сторонами интерфейса.

## Координаты и время

SI (Système international d’unités — Международная система единиц) units, правосторонние системы ROS (Robot Operating System — программная платформа для робототехники): `x` вперёд, `y` влево, `z` вверх; yaw в радианах. TF (Transform library — библиотека преобразований между системами координат): `map → odom → base_link → lidar_link / imu_link / camera_link → camera_optical_frame`. Optical frame: `z` вперёд, `x` вправо, `y` вниз. G1 владеет двумя динамическими transforms, simulator adapter — только sensor static transforms. Исключить duplicate TF publishers от Webots/Nav2 (Navigation 2 — стек навигации для второй версии платформы Robot Operating System).

Единственный источник `/clock` — симулятор через bridge. Runtime nodes используют `use_sim_time=true`. Latency вычисляется steady clock; возраст измерения — simulation time. При reset времени очистить фильтры, histories, action states, cached TF и команду; episode ID (Identifier — идентификатор) меняется. Watchdog остановки использует также steady clock, чтобы зависший `/clock` не скрывал отказ.

## Topics и actions

| Интерфейс | ROS 2 (Robot Operating System 2 — вторая версия программной платформы для робототехники) type | Owner → consumer | Rate / deadline | QoS (Quality of Service — политики качества обслуживания при передаче сообщений) / условие |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | simulator → all | progressing | best effort, volatile, depth 10 |
| `/sensors/wheel_odom` | `nav_msgs/msg/Odometry` | simulator → G1 | 50 Hz (Hertz — герц, число циклов в секунду) | sensor data QoS |
| `/sensors/imu` | `sensor_msgs/msg/Imu` | simulator → G1 | 50 Hz | sensor data QoS |
| `/sensors/scan` | `sensor_msgs/msg/LaserScan` | simulator → G1/G4 | 10 Hz | sensor data QoS |
| `/sensors/rgb` | `sensor_msgs/msg/Image` | simulator → G2 | 10 Hz | sensor data QoS, `rgb8` |
| `/sensors/depth` | `sensor_msgs/msg/Image` | simulator → G2 | 10 Hz | sensor data QoS, `32FC1`, m |
| `/sensors/camera_info` | `sensor_msgs/msg/CameraInfo` | simulator → G2 | 10 Hz | sensor data QoS |
| `/state/odom` | `nav_msgs/msg/Odometry` | G1 → G4/G5 | 50 Hz / 20 ms (Millisecond — миллисекунда, одна тысячная секунды) | reliable, volatile, depth 5; frame `odom`, child `base_link` |
| `/localization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | G1 → G3/G5 | 10 Hz / 100 ms | reliable, volatile, depth 5; frame `map` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | G1/map server → G3 | once per episode | reliable, transient local, depth 1; frame `map` |
| `/perception/obstacles` | `nav_msgs/msg/OccupancyGrid` | G2 → G3/G4 | 10 Hz / 100 ms | reliable, volatile, depth 1; frame `odom` |
| `/planning/compute_path` | `nav2_msgs/action/ComputePathToPose` | G5 client → G3 server | on demand; budget 200 ms | standard ROS action QoS; cancellation supported |
| `/control/follow_path` | `nav2_msgs/action/FollowPath` | G5 client → G4 server | loop 20 Hz / 50 ms | standard action QoS; feedback and cancel |
| `/control/cmd_vel_raw` | `geometry_msgs/msg/TwistStamped` | G4 controller → monitor | 20 Hz | reliable, volatile, depth 1; frame `base_link` |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | G4 monitor → simulator adapter | 20 Hz | reliable, volatile, depth 1 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | all → logger/G5 | 1 Hz + events | reliable, volatile, depth 10 |
| `/mission/status` | `diagnostic_msgs/msg/DiagnosticArray` | G5 → logger | events + 1 Hz | reliable, volatile, depth 10 |
| `/evaluation/ground_truth` | `nav_msgs/msg/Odometry` | simulator → evaluator only | 50 Hz | reliable, volatile, depth 10; frame `map` |

Budgets относятся к обработке одного входа/цикла на выбранной машине, не к времени успешной доставки. Зафиксировать CPU (Central Processing Unit — центральный процессор)/GPU (Graphics Processing Unit — графический процессор)/thread count и измерять очереди отдельно. Ни одна группа не должна молча менять rate, frame или тип сообщения. Для будущего Nav2 plugin deployment эти topics remap-ятся явно; stock Nav2 bringup не предполагается автоматически совместимым.

Grid: row-major, cell index `y * width + x`, origin pose задаёт угол и начало сетки; 0 = free, 100 = occupied, −1 = unknown. В планировании unknown блокируется, если иной единый режим не зафиксирован до test. G2 выдаёт **неинфлированную** сетку; G3/G4 применяют один footprint + margin. Нельзя складывать двойную inflation.

## Порядок миссии и пути

G5 запрашивает G3 через `ComputePathToPose`, получает path и передаёт его в G4 через `FollowPath`. G4 отвечает feedback/progress, success только при pose tolerance 0.25 м, yaw tolerance 0.2 рад и low-speed stop. При no-path G5 выполняет bounded wait/replan; при controller failure — cancel/stop и не более двух recovery attempts. Изменение destination отменяет предыдущий action. Только G4 monitor имеет право публиковать `/cmd_vel`.

## Обработка отказов

| Fault | Требуемая реакция | Owner |
|---|---|---|
| Нет новой raw command >150 ms sim time либо >300 ms steady time | Нулевая команда, diagnostic, без автоматического возобновления старой команды | G4 |
| Нет scan/odom >300 ms sim time | Stop; recovery лишь после свежих валидных данных | G4 |
| Нет perception >300 ms | Stop и replan после восстановления; не ехать по старой карте | G2/G4/G5 |
| Некорректная covariance, NaN (Not a Number — специальное значение «не число»)/Inf (Infinity — специальное значение бесконечности), invalid model | Reject output, classical fallback либо stop, log reason | каждая группа |
| Planning >200 ms / no-path | Явный failure/cancel; bounded retry | G3/G5 |
| Control cycle >50 ms | Deadline counter; monitor не принимает stale command | G4 |
| Simulator driver не получает `/cmd_vel` >300 ms steady time | Stop на уровне driver/adapter | G1 с G4 |
| Time reset или TF mismatch | Stop, clear state, readiness handshake | все |

Stopping envelope включает измеренную задержку, `v²/(2*a_brake)` и margin. Выбранное `a_brake` обосновать динамикой симулятора. Простая проверка дистанции фиксированным радиусом не заменяет stopping model. Simulator contact events используются evaluator для collision detection, а не monitor как подсказка будущего столкновения.

Каждый компонент имеет `configure → ready → active → fault/stopped`; предпочтительны `rclcpp_lifecycle` nodes. G5 активирует автономию только после readiness всех входов. Ground truth и labels имеют отдельный namespace и запрещены runtime imports/subscriptions; launch/ROS graph audit входит в приёмку.

## Контрактный тест каждой группы

Подать записанный валидный вход, проверить type/frame/stamp/units, частоту и deadline; затем stale, missing и nonfinite input. Сравнить A/B outputs по одной schema; проверить switch только между эпизодами. Команда меняет метод через конфигурацию `method: classical|learned`, не заменой исходников. Зафиксировать доступные fallback modes и их срабатывания в manifest.
