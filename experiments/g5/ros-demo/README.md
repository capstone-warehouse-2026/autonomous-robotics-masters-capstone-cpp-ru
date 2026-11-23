# Проверка G5 через отдельные ROS-процессы

Это отладочная кинематика и реальные ROS actions, не Webots и не реализация G1–G4.
Запуски выполнены в отдельных ROS_DOMAIN_ID 175–179, ROS 2 Jazzy, native C++.
Журналы содержат все переходы, `episode.yaml` — итог runner,
`metrics.csv` — результат C++ `mission_metrics`, `exit_code.txt` — код всего launch.

| Каталог | Условие | Итог | Доставлено / поступило | Код launch |
|---|---|---|---:|---:|
| classical | EDD | completed | 2 / 2 | 0 |
| learned | выбранный по validation DQN | completed | 2 / 2 | 0 |
| blocked | `blocked_route:=true` | mission_fault | 0 / 2 | 1 |
| controller_failure | `controller_failure:=true` | mission_fault | 0 / 2 | 1 |
| frozen | `freeze_clock=true` во время FollowPath | frozen_clock | 0 / 2 | 1 |

При frozen action не выдал конечного результата: G5 записал
`episode_fault: action_result_unknown`, а не подтвердил отмену. Сценарий frozen
увеличивает расстояние до pickup до10м, чтобы внести отказ до окончания движения.
Параметр изменён CLI после появления `following_started`; `injection.log`
подтверждает ответ сервиса параметров.

Обычный запуск из собранной среды (каталог output должен быть новым):

```bash
ros2 launch capstone_benchmark debug_episode.launch.py \
  output:=/tmp/g5-run-unique method:=classical domain_id:=175
ros2 launch capstone_benchmark debug_episode.launch.py \
  output:=/tmp/g5-run-learned-unique method:=learned domain_id:=176
```

Для блокировки добавить `blocked_route:=true`, для отказа управления —
`controller_failure:=true`. Ожидается ненулевой код launch; это успешная проверка
реакции на отказ, но не успешная доставка. Для frozen запустить с
`manifest:=<absolute-path>/frozen/scenario.yaml`, затем в том же ROS_DOMAIN_ID:

```bash
ros2 param set /debug_navigation freeze_clock true
```

Каталог `tabular` содержит C++ запуск того же demo manifest через изолированную
среду. Оба метода доставляют2/2 вовремя и проходят3.4142м. Табличная длительность
8.8284с, ROS —7.4с для EDD и7.7с для DQN. Причины ожидаемого различия: табличная
геометрическая оценка использует0.5м/с, debug-движение1м/с, а ROS дополнительно
тратит время на actions, подтверждение остановки и дискретные BT ticks. Это один
контрольный эпизод; разницу методов по нему статистически оценивать нельзя.
Проверка переноса в полную физическую симуляцию остаётся отдельным экспериментом.

`source_snapshot.sha256` фиксирует исходники и модели проверенного состояния;
`native_provenance.yaml` — аппаратную среду и версии ROS. Запуски выполнены с
незакоммиченными изменениями (`dirty: true`): `code_commit` обозначает базовый
коммит, точное проверенное состояние определяется snapshot, а не только HEAD.
