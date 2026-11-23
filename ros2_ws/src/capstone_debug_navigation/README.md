# Отладочная навигация для G5

`debug_navigation` — независимая кинематическая среда на C++ для проверки G5 через
настоящие ROS actions. Она не заменяет G1–G4 и не доказывает качество навигации в
Webots: прямой путь не проверяет геометрию препятствий, а odom/map совпадают по
допущению стенда. Другие пакеты навигации не изменяются.

Публикуются `/clock`, `state/odom`, `localization/pose`, `diagnostics`; серверы
`planning/compute_path` и `control/follow_path` поддерживают отмену и feedback.
`debug/reset` завершает активные actions, возвращает робота в начало и сбрасывает
время. Мир не запускается одновременно с другим источником `/clock`.

```bash
make build PKG=capstone_debug_navigation
make build PKG=capstone_mission
make shell
export ROS_DOMAIN_ID=75
ros2 launch capstone_debug_navigation demo.launch.py
```

Изменение метода: `method:=learned model:=/absolute/path/to/dispatch_dqn.json`.
Путь manifest задаётся `manifest:=...`, журнал — `event_log:=...`.

Узлу доступны параметры `speed` (м/с), `time_scale` (ускорение времени).
Во время работы можно менять параметры отказов:

```bash
ros2 param set /debug_navigation blocked_route true
ros2 param set /debug_navigation controller_failure true
ros2 param set /debug_navigation freeze_clock true
ros2 param set /debug_navigation publish_odom false
ros2 param set /debug_navigation failure_to_stop true
ros2 param set /debug_navigation planning_delay 0.5
```

Последний параметр измеряется реальным временем и проверяет deadline планирования.
`failure_to_stop` сохраняет движение после принятой отмены: G5 должен обнаружить
отсутствие остановки по odom, а не объявить успешную отмену по одному ответу action.
Запрос `debug/reset` возвращает время/позу, но намеренно не изменяет настройки отказов.
