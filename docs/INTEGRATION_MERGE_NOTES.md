# Интеграция реальных веток: протокол и результаты

Дата: 25.11.2026. Ветка: `integration/merge`. Автор: Волков Андрей (`buigbvns`).

## Что слито

`integration/merge` = `main` + `origin/g2/classical` + `origin/G1` + `origin/G3` +
`origin/integration/skeleton` (стенд). Слияние прошло без конфликтов.

| Пакет | Тесты после слияния |
|---|---|
| `capstone_localization` (G1) | 17/17 |
| `capstone_planning` (G3) | 45/45 |
| `capstone_perception` (G2) | 37/37 |
| `capstone_integration` (стенд) | 60/60 |

Не сливались: `g4/ppo` (отстаёт от `main` на 14 коммитов и удаляет MPC и файлы G2),
`integration/g5-nav2-webots` (отдельный большой стек миссии/Nav2/флота).

## Запуск с настоящими компонентами

Образ симулятора пересобран с датчиками G1: `/sensors/imu`, `/sensors/scan`,
`/sensors/wheel_odom`, `/evaluation/ground_truth`.

```bash
ros2 launch capstone_integration system.launch.py \
  start_localization:=true publish_stub_localization:=false \
  start_planning:=true start_path_stub:=false \
  start_perception:=true episode_id:=real-1
```

Для этого в стенд добавлено:

- `world_stub` отключает свои датчики/одоверию/позу/TF (`publish_localization=false`) и публикует
  только опорную статическую карту со стеллажами и паллетами из `warehouse.wbt`;
- `drive_stub` переводит путь из `map` в `odom` через TF (без этого команды уезжают на 8 м);
- миссия ждёт 2 с перед первой заявкой и делает до 5 повторных запросов пути (гонка с `/map`).

## Аудит контрактов на живом стеке

```text
/sensors/scan        FAIL   frame 'lidar_link', expected 'base_link'
/localization/pose   WARN   41.0 Hz, expected about 10.0 Hz
/perception/obstacles WARN  6.4 Hz, expected about 10.0 Hz
placeholders         WARN   drive_stub, mission_stub, world_stub
остальные 17 проверок — OK (включая /sensors/imu, TF map->base_link,
                              ровно один publisher /cmd_vel, оба action-сервера)
```

## Результат прогона: миссия не завершилась

Через ~5 с после начала движения safety выдал `COLLISION_RISK` и остановил робота.
Сравнение поз в этот момент:

| Источник | x, м | y, м |
|---|---:|---:|
| `/evaluation/ground_truth` | -9.62 | 4.82 |
| `/localization/pose` (G1) | -8.42 | 0.79 |

Оценка G1 отстала от реальности примерно на 4 м по пути. Контроллер вёл робота по неверной позе,
робот физически ушёл к западной стене/паллете, safety корректно остановил его по лидару. CSV эпизода
не записан: `FollowPath` не завершился, миссия ждала до остановки прогона.

Дополнительно в логах G2: `floor not found (0 candidates)` и `frame dropped: invalid or stale
RGB-D timestamp` — восприятие под нагрузкой теряет пол и кадры.

## Выводы

1. Стенд и контракты работают: реальные G1/G2/G3/G4-safety поднимаются вместе, аудит находит
   конкретные расхождения.
2. Блокер сквозного C0 — **расхождение локализации G1** на новой симуляции; без его устранения
   миссия не может завершиться.
3. Дефекты для передачи группам: frame скана (`lidar_link` вместо `base_link`), частота
   `/localization/pose` 41 Гц вместо 10, дрейф локализации ~4 м, пропуски кадров G2.
4. У миссии нет recovery: остановка safety превращается в зависание до таймаута; это задача G5.

## Воспроизведение

```bash
cd ~/.cache/capstone-merge          # рабочее дерево ветки
make build && docker compose build simulator
docker compose up -d simulator && sleep 25
docker compose run -d --name real1 dev bash -c \
  "source /etc/capstone_dev.sh && ros2 launch capstone_integration system.launch.py \
   start_localization:=true publish_stub_localization:=false start_planning:=true \
   start_path_stub:=false start_perception:=true episode_id:=real-1"
docker exec real1 bash -c "source /etc/capstone_dev.sh && ros2 run capstone_integration graph_audit"
```
