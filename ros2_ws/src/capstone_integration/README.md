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
| `/mission/status`, 6 заявок, CSV | **PLACEHOLDER G5** (earliest due date) |

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

Что произойдёт: робот поедет от стартовой точки `(-8, 0)` по шести заявкам манифеста
`config/jobs.csv` — кольцом по станциям склада, с остановкой на 2 секунды на погрузке и разгрузке,
и запишет строку результата.

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
stub-001,SKELETON,...,SUCCESS,208.400,6,...
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
`docs/INTERFACES.md`), время обслуживания `2 с`, лимит эпизода `300 с`. Заявки — в
`config/jobs.csv` (id, погрузка, разгрузка, время появления, срок); пустой `jobs_file` включает
встроенный манифест из шести заявок.

## Ограничения стенда (важно)

- Локализация — **dead reckoning** по `/cmd_vel`: накапливает ошибку, это не EKF/ICP.
- `/sensors/scan` — **пустой** скан: препятствий нет, safety не срабатывает по препятствиям.
- Планировщик **игнорирует** карту препятствий и едет по прямой; может проехать через стеллаж.
- Миссия — **последовательная** (вместимость 1): earliest due date, без Behavior Tree, без
  прерывания активной доставки и без recovery; заявки выдаются по одной.
- Сброс эпизода — **перезапуск контейнера симулятора**, а не reset через supervisor: медленно
  (~25 с) и не очищает TF/истории внутри узлов, потому что узлы тоже перезапускаются.
- Отказы: автоматизированы 3 из 5, один повтор; заморозка `/clock` и битая модель проверяются
  вручную, задержка реакции на них пока не пишется в CSV.
- Расстояние в CSV берётся из длины маршрута, а не из пройденного пути.

Всё это заменяется по одному узлу: когда группа приносит свой узел, он запускается вместо
соответствующего `*_stub` (флаг `start_mission`, отдельные `Node(...)` в `launch/system.launch.py`).

## Аудит контрактов

Пока система запущена, отдельным терминалом:

```bash
ros2 run capstone_integration graph_audit --ros-args \
  -p window_s:=5.0 -p json_path:=/workspace/results/audit.json
```

Узел ничего не публикует и не двигает: он слушает граф 5 секунд и сверяет его с таблицей
`docs/INTERFACES.md` (топик, тип, число publisher'ов, частота, `frame_id`, TF, action-серверы,
подписка runtime-узлов на ground truth, лишние топики). Результат: таблица в терминал, JSON в файл,
`/diagnostics`, код возврата 1 при наличии FAIL (`-p fail_on:=never` отключает).

Замер на стенде (эпизод `audit-002`, окно 5 с):

```text
/clock                           OK      advancing
/sensors/wheel_odom              OK      50.0 Hz, frame 'odom'
/sensors/imu                     FAIL    0 publisher(s), expected >= 1
/sensors/scan                    OK      10.0 Hz, frame 'base_link'
/sensors/rgb                     WARN    6.8 Hz, expected about 10.0 Hz
/state/odom                      OK      50.0 Hz, frame 'odom'
/localization/pose               OK      10.0 Hz, frame 'map'
/cmd_vel                         OK      20.2 Hz, frame 'base_link'
/planning/compute_path           OK      action server present
/control/follow_path             OK      action server present
TF map->base_link                OK      chain resolves
placeholders                     WARN    placeholder nodes active: ...
ground_truth_leak                OK      no runtime subscriber
```

`-p measured_run:=true` превращает активные заглушки в FAIL: результаты такого прогона нельзя
считать системными.

## Серия эпизодов и агрегация

Сброс эпизода сейчас = перезапуск контейнера симулятора (свежий мир, робот на стартовой позе).
Отдельного reset-сервиса у Webots через ROS нет, поэтому серию запускает хост-скрипт:

```bash
# 3 эпизода, каждый со свежим симулятором; результаты в results/
ros2_ws/src/capstone_integration/tools/run_series.sh 3 results

# эпизод как «измеряемый»: заглушки обязаны провалить аудит (скрипт вернёт 1)
ros2_ws/src/capstone_integration/tools/run_series.sh 1 results 1

# продолжить серию с эпизода 4, не перезаписывая первые три
START_AT=4 ros2_ws/src/capstone_integration/tools/run_series.sh 20 results
```

Каждый эпизод получает три файла:

| Файл | Содержимое |
|---|---|
| `episode.csv` | строка метрик эпизода |
| `audit.json` | снимок аудита контрактов во время эпизода |
| `manifest.json` | commit, id образа, config/scenario, `measured_run`, код возврата аудита, время |

Сводка по аудитам — `results/series_audit.csv`:

```text
episode_id,measured,failures,warnings,audit_exit
series-1,false,1,5,1
```

Если `measured=1` и аудит красный, эпизод помечается `INVALID`, а скрипт завершается с кодом 1:
такой прогон нельзя предъявлять как системный результат.

```bash
# сводка по всем results/*/episode.csv
make shell
ros2 run capstone_integration benchmark_aggregate /workspace/results
```

Агрегатор (C++, без ROS) пишет `results/summary.csv` и печатает таблицу с долей успеха,
интервалом Уилсона 95 %, средним и p95 времени, средним путём.

Пример на семи эпизодах стенда:

```text
CONFIG    SCEN  N  OK  RATE  95% CI       TO  AB  MEAN_S  P95_S  DIST_M
SKELETON  S1    7   7  1.00  0.65..1.00    0   0   17.98  18.60    7.54
```

Интервал Уилсона при 7/7 широкий честно: для сужения нужна выборка из задания (20 эпизодов на
случай). Заглушки в `placeholder_components` напоминают, что это не системные результаты.

## Отказы

```bash
# три автоматических отказа: датчик, устаревшая команда, падение контроллера
ros2_ws/src/capstone_integration/tools/run_faults.sh 1 results
```

Инжектор ждёт, пока робот поедет, вносит отказ, замеряет задержку до нулевой команды и до первого
WARN/ERROR в `/diagnostics`, затем снимает отказ и проверяет восстановление. Результат —
`results/faults/faults.csv`:

```text
run_id                 fault             stop_s  diag_s  diag_reason          recovered
fault-sensor_dropout-1 sensor_dropout    0.347   0.326   STALE_SCAN           true
fault-controller_pause controller_pause  0.203   0.163   STALE_CMD            true
fault-controller_crash controller_crash  0.153   0.132   STALE_CMD            false
```

Ещё два отказа проверяет отдельный скрипт (ему нужен свободный симулятор):

```bash
ros2_ws/src/capstone_integration/tools/check_manual_faults.sh results
```

- **Битая модель.** `ros2 run capstone_perception perception_node --ros-args -p method:=learned
  -p learned.model_path:=/nonexistent.onnx` — узел не падает и на каждый кадр пишет
  `WARN ... using the classical fallback`. Резерв работает, но `fallback_count` в CSV пока не
  заполняется.
- **Заморозка `/clock`.** `docker pause ...-simulator-1` на 4 с во время движения: `/cmd_vel`
  остаётся `≈0.50`, новых `halt` в логе нет; после `unpause` движение продолжается. Это **дефект
  G4**: safety не останавливает робота при зависшем времени симуляции. Задержка пишется как `NA`,
  разрешение проверки — секунды.

Полный набор по заданию — 5 отказов × 5 повторов × C0/C6; здесь автоматизированы 3 из 5 и один
повтор на демонстрации. C6 требует настоящих обученных компонентов G1/G3/G5.

## Тесты

```bash
make test PKG=capstone_integration
```

Проверяются: интегрирование движения, построение и следование пути, формат CSV, публикация
контракта датчиков.
