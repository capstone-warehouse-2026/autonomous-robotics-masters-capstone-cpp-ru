# Nav2 baseline для интеграции G5 с Webots

Пакет адаптирует конфигурацию `ware_ws/src/waf_navigation`: глобальный Navfn A*
и локальный Regulated Pure Pursuit (RPP). Проверяемая локальная версия Nav2 —
Jazzy 1.3.14. Это готовая навигационная основа для настоящего движения G5;
она не подменяет собственные EKF/ICP, A*/CNN, MPC/PPO из заданий G1–G4.

## Поток данных

```text
Webots encoder odometry → state/odom, odom→base_link
Webots LaserScan → sensors/scan
static map + odometry + scan → AMCL → map→odom, amcl_pose
amcl_pose → contracts adapter → localization/pose (исходный stamp)
scan + TF → contracts adapter → perception/obstacles (odom, без inflation)
G5 → planning/compute_path → Navfn A*
G5 → control/follow_path → RPP → nav2/cmd_vel (Twist)
contracts adapter → control/cmd_vel_raw (TwistStamped)
capstone_safety → cmd_vel (TwistStamped) → Webots wheels
```

`bt_navigator`, velocity smoother и recovery behavior server не запускаются:
дерево миссии, recovery и отмена принадлежат G5. `/cmd_vel` публикует только
`capstone_safety`; adapter не обходит защитный монитор. Существующий driver
watchdog остаётся последним уровнем остановки при исчезновении команд.

Серверы используют штатные `nav2_msgs/action/ComputePathToPose` и `FollowPath`.
Имена actions remap-ятся на контракты G5. RPP работает 20 Гц, целевая скорость
0.35 м/с, разворот 0.6 рад/с. StoppedGoalChecker требует остановки ниже
0.05 м/с и 0.1 рад/с, допуски позы 0.20 м и 0.20 рад. PoseProgressChecker
различает движение и разворот и допускает 15 секунд без прогресса.

## Локализация и свежесть

AMCL использует реальный scan и encoder odometry. Начальная гипотеза
`(-3, 0, yaw=0)` задаётся известным стартом интеграционного сценария;
runtime ground truth не подписывается ни AMCL, ни adapter, ни G5.
После обратного скачка `/clock` adapter восстанавливает ту же начальную гипотезу
только после возобновления свежего TF. Смена места старта требует одинаковых
`initial_x`, `initial_y`, `initial_yaw` в мире/launch.

На неподвижном роботе AMCL обычно ждёт перемещения. Adapter асинхронно вызывает
`request_nomotion_update` при scan и каждые 100 мс, чтобы AMCL использовал новое
лазерное измерение. Один запрос остаётся в работе не более секунды steady time.
Старые pose/scan не получают новых timestamps: при возрасте более 300 мс они
отклоняются, G5 и safety обнаруживают потерю данных. Команда Twist получает stamp
единожды при поступлении из controller_server, затем не переиздаётся таймером.

Карта `perception/obstacles` строится из scan: пересечённые лучами клетки свободны,
конечные точки заняты, невидимые клетки неизвестны. Никакой inflation здесь нет.
Nav2 costmaps самостоятельно используют тот же scan и одну inflation layer.
Эта геометрическая основа обеспечивает согласованный интерфейс до подключения
полноценного G2; G2 и adapter не должны одновременно публиковать этот топик.

## Запуск

Сначала запускается Webots bridge с encoder odometry, scan и `/clock`, затем:

```bash
ros2 launch capstone_nav2 navigation.launch.py \
  map:=/absolute/path/to/simulation/integration/map.yaml
```

Launch включает `map_server`, `amcl`, `planner_server`, `controller_server`,
два lifecycle manager, `nav2_contracts` и существующий `safety_monitor`.
Не запускайте второй safety monitor рядом. Параметры карты обязаны совпадать
с геометрией мира; `map` — обязательный аргумент. Возможны `params_file`,
`initial_x`, `initial_y`, `initial_yaw`.

Подключение G5 выполняется его обычным launch с реальным сценарием и
`require_safety:=true`. Тестовый `capstone_debug_navigation` рядом не запускается.
При запуске в namespace внешние topics/TF необходимо remap-ить согласованно;
этот интеграционный профиль рассчитан на одного робота без namespace.

## Проверки

`make test PKG=capstone_nav2` проверяет лучевую карту, метки времени, NaN/Inf,
проекцию в odom и отсутствие переиздания старых команд/AMCL-поз. ROS-тесты
следует выполнять в отдельном `ROS_DOMAIN_ID`. Python launch проверяется
`ament_flake8 ros2_ws/src/capstone_nav2/launch`.

Полная проверка Nav2 требует реального Webots процесса: создание сервера actions
или прохождение unit tests не доказывает прохождение маршрута. Результаты таких
запусков сохраняются отдельно от изолированной табличной серии G5.
