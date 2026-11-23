# `capstone_integration` — интеграционный стенд

Сокращения раскрыты при первом употреблении; [общий словарь](../../../docs/GLOSSARY.md).

Пакет ничего не заменяет навсегда: он делает **стыки между группами рабочими уже сейчас**, пока
G1/G3/G4/G5 не сдали свои узлы. Каждый временный узел помечен `PLACEHOLDER` в логе и в файле
результата. Это не методы групп и их нельзя предъявлять как их результаты.

## Что реально, а что временно

| Компонент | Источник |
|---|---|
| Симулятор, камера, привод | штатные пакеты проекта |
| Восприятие `/perception/obstacles` | настоящий узел G2 (`capstone_perception`) |
| Safety monitor, `/cmd_vel` | настоящий узел G4 (`capstone_safety`) |
| `/sensors/scan`, `/sensors/wheel_odom` | **PLACEHOLDER G1** |
| `/state/odom`, `/localization/pose`, `/map`, TF | **PLACEHOLDER G1** (dead reckoning) |
| `/planning/compute_path` | **PLACEHOLDER G3** (прямая линия) |
| `/control/follow_path`, `/control/cmd_vel_raw` | **PLACEHOLDER G4** (pure pursuit) |
| `/mission/status`, одна заявка, CSV | **PLACEHOLDER G5** |

## Запуск

Терминал 1 — симулятор:

```bash
make sim-gui          # или make sim без окна
```

Терминал 2 — система:

```bash
make shell
ros2 launch capstone_integration system.launch.py
```

Что произойдёт: робот начнёт движение от стартовой точки `(-8, 0)` к станции `STATION_1`
`(-8.8, 7.5)`, остановится там на 2 секунды и запишет результат.

Ожидаемые строки в логе:

```text
PLACEHOLDER G1: dead-reckoning sensors ...
PLACEHOLDER G3: straight-line planner ...
PLACEHOLDER G4: pure-pursuit follower ...
PLACEHOLDER G5: one fixed job ...
episode stub-001 finished: SUCCESS
episode written to /workspace/results/stub-001/episode.csv
```

Результат (на хосте `results/stub-001/episode.csv`):

```text
episode_id,config_id,...,terminal_reason,elapsed_sim_s,delivered,...
stub-001,SKELETON,...,SUCCESS,38.420,1,...
```

Полезные варианты запуска:

```bash
# без настоящего восприятия (быстрее, если камера не нужна)
ros2 launch capstone_integration system.launch.py start_perception:=false

# другой эпизод и папка результатов
ros2 launch capstone_integration system.launch.py episode_id:=dbg-2 results_dir:=/workspace/results

# без миссии: поднять только датчики/локализацию/планировщик/контроллер
ros2 launch capstone_integration system.launch.py start_mission:=false
```

## Параметры

Все пороги — в `config/system.yaml`: стартовая поза, частоты, допуски цели (`0.25 м`, `0.2 рад` по
`docs/INTERFACES.md`), время обслуживания `2 с`, лимит эпизода `300 с`, координаты станции.

## Ограничения стенда (важно)

- Локализация — **dead reckoning** по `/cmd_vel`: накапливает ошибку, это не EKF/ICP.
- `/sensors/scan` — **пустой** скан: препятствий нет, safety не срабатывает по препятствиям.
- Планировщик **игнорирует** карту препятствий и едет по прямой; может проехать через стеллаж.
- Миссия — **одна фиксированная заявка**, без очереди, сроков и Behavior Tree.
- Расстояние в CSV берётся из длины маршрута, а не из пройденного пути.

Всё это заменяется по одному узлу: когда группа приносит свой узел, он запускается вместо
соответствующего `*_stub` (флаг `start_mission`, отдельные `Node(...)` в `launch/system.launch.py`).

## Тесты

```bash
make test PKG=capstone_integration
```

Проверяются: интегрирование движения, построение и следование пути, формат CSV, публикация
контракта датчиков.
