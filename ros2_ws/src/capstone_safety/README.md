# `capstone_safety` — G4: независимый safety monitor

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Задание: [projects/04_control.md](../../../projects/04_control.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md).

Узел `safety_monitor` — **единственный** publisher `/cmd_vel`. Он одинаков для обоих методов управления (A — MPC, B — PPO) и пропускает команду контроллера только если все проверки пройдены; иначе публикует ноль. Monitor не заменяет корректность контроллера: он ловит отказы, а не улучшает управление.

| | Топик | Тип | QoS |
|---|---|---|---|
| Вход | `/control/cmd_vel_raw` | `geometry_msgs/msg/TwistStamped`, frame `base_link` | reliable, volatile, depth 1 |
| Вход | `/state/odom` | `nav_msgs/msg/Odometry` | reliable, volatile, depth 5 |
| Вход | `/sensors/scan` | `sensor_msgs/msg/LaserScan` + TF `base_link ← <frame скана>` | sensor data |
| Выход | `/cmd_vel` | `geometry_msgs/msg/TwistStamped`, frame `base_link`, 20 Гц | reliable, volatile, depth 1 |
| Выход | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray`, статус `capstone_safety/safety_monitor` | при смене причины + 1 Гц |

## Проверки (в этом порядке, первая сработавшая останавливает робота)

| Причина | Условие |
|---|---|
| `TIME_JUMP` | stamp входа из будущего (сброс симуляции) → состояние очищается |
| `NOT_READY` | ещё нет команды, odom или скана |
| `STALE_SCAN`, `STALE_ODOM` | вход старше 300 мс sim time |
| `STALE_CMD` | команда старше 150 мс sim **или** 300 мс steady time; latch до прихода команды с более новым stamp |
| `NONFINITE` | NaN/Inf в команде |
| `LIMIT` | `v ∉ [0, 0.6]` или `abs(ω) > 1.2` (допуск 0.02) — остановка, а не обрезка |
| `COLLISION_RISK` | точка скана в зоне `robot_radius + margin` вдоль тормозного пути `v·t_latency + v²/(2·a_brake)` при `v = max(v_cmd, v_odom)` |

Ноль публикуется на **каждом** тике, пока причина не исчезнет. Пороги — в [config/safety.yaml](config/safety.yaml).

## Lifecycle

`unconfigured → (configure) → inactive → (activate) → active`.

- **configure** читает параметры (ошибка → узел остаётся unconfigured), создаёт подписки, publishers, TF.
- **activate** сбрасывает ядро (данные до активации не используются) и запускает таймер 20 Гц.
- **deactivate** публикует нулевую команду и замолкает; робота держит watchdog драйвера (300 мс).
- Пока узел не `active`, `/cmd_vel` не публикуется вообще.

`autostart:=true` (по умолчанию) — узел сам проходит configure и activate. `autostart:=false` — переходами управляет G5:

```bash
ros2 lifecycle set /safety_monitor configure
ros2 lifecycle set /safety_monitor activate
ros2 lifecycle get /safety_monitor
```

## Структура

- `include/capstone_safety/safety_core.hpp`, `src/safety_core.cpp` — чистая логика без ROS (`SafetyCore`).
- `include/capstone_safety/safety_monitor_node.hpp`, `src/safety_monitor_node.cpp` — lifecycle-узел, `scan_to_points`.
- `src/main.cpp` — запуск и autostart; `launch/safety.launch.py`; `config/safety.yaml`.
- `test/test_safety_core.cpp` — логика ядра; `test/test_safety_monitor_node.cpp` — узел через настоящие топики.

## Команды

```bash
make test PKG=capstone_safety
make shell
ros2 launch capstone_safety safety.launch.py use_sim_time:=false
ros2 topic hz /cmd_vel                       # ≈ 20 Гц
ros2 topic echo /diagnostics --once
```
