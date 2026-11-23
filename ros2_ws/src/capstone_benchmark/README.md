# G5: воспроизводимые эксперименты на C++

`dispatch_benchmark` выполняет изолированные табличные эпизоды с тем же C++ кодом
очереди, observation, маски, EDD и DQN, который использует ROS-узел. Это сравнение
диспетчеризации, не измерение поведения полного робота. Таблицы `train`, `validation`,
`test` разделены в `ml/mission/scenarios`. Тест содержит пять семейств ×20 эпизодов.

```bash
make build PKG=capstone_benchmark
make test PKG=capstone_benchmark
make shell
ros2 run capstone_benchmark dispatch_benchmark \
  --scenarios /workspace/ml/mission/scenarios/test \
  --output /workspace/results/g5/example \
  --model seed0=/workspace/ros2_ws/src/capstone_mission/models/dqn_seed_0.json \
  --ablation ablation0=/workspace/ros2_ws/src/capstone_mission/models/dqn_no_congestion_seed_0.json \
  --commit YOUR_COMMIT_SHA
ros2 run capstone_benchmark aggregate_dispatch \
  /workspace/results/g5/example/metrics.csv /workspace/results/g5/example/summary.csv
```

Повторите `--model`/`--ablation` для всех трёх training seeds. Классический метод
всегда добавляется один раз. Существующий каталог с `metrics.csv` не перезаписывается.
Модель загружается до эпизода; некорректная модель приводит к явному fallback,
его причина сохраняется в manifest, количество — в metrics.

Выход: `metrics.csv`, `events.csv`, `manifest.yaml`, копии сценариев. Manifest
содержит SHA256 моделей/сценариев, seed, split, compiler и переданный commit.
Доставки в срок делятся на **все поступившие** заявки; unfinished включает failed.
При нуле доставок расстояние/доставку и tardiness записываются как `NA`. Расстояние
считается по реально пройденной части табличного маршрута. Блокировка даёт две
попытки по секунде, затем failed. Отказ после получения груза запрещает следующую
доставку; поступившие до горизонта заявки остаются в знаменателе.

Агрегатор проверяет schema, дубли и полноту пар внутри семейства. Единица анализа —
эпизод; CI95 и парные differences к EDD считаются bootstrap 10000 выборок, seed2026.
Training seeds выводятся отдельно и не выдаются за дополнительные независимые эпизоды.

`benchmark_runner_node` — ROS-супервизор отдельного эпизода:
reset симулятора → reset миссии → свежие clock/odom → start → terminal или отмена.
Он работает с `mission/start`, `mission/reset`, `mission/cancel` и `/mission/status`;
параметр `simulation_reset_service` позволяет заменить отладочный `debug/reset`.
Миссию необходимо запускать с `autostart:=false`. Включение runner:

```bash
ros2 run capstone_benchmark benchmark_runner_node --ros-args \
  -p run_episode:=true -p output_directory:=/tmp/g5-run-unique \
  -p use_sim_time:=true -p horizon:=300.0
```

Runner сохраняет `episode.yaml`: причина завершения, simulation duration,
наблюдённое расстояние, commit и явно указанная область проверки. Отсутствие входов,
замершие часы и timeout приводят к отмене; принятие запроса отмены не считается её
завершением. Итоговые delivery metrics пересчитывает C++ `mission_metrics` из журнала миссии
и manifest; табличный runner отдельно выполняет расчёт для изолированной среды. Общая серия C0–C6
требует готовых компонентов остальных групп и не подменяется отладочным стендом.

Для отладочного стенда есть единая команда, которая также вычисляет delivery
metrics, останавливает все процессы и возвращает ненулевой exit code при отказе:

```bash
ros2 launch capstone_benchmark debug_episode.launch.py \
  output:=/tmp/g5-unique-run method:=classical domain_id:=175
```

После runner C++ `mission_metrics` проверяет соответствие episode ID, известность
заявок и порядок `selected → picked → delivered`, затем считает metrics по ledger
и исходному manifest. Отдельный пересчёт:

```bash
ros2 run capstone_benchmark mission_metrics scenario.yaml events.csv episode.yaml metrics.csv
```

Агрегатор полной табличной серии требует пять семейств ×20 эпизодов для каждого
метода и сохраняет дополнительный `summary_all_metrics.csv` с доверительными
интервалами всех числовых метрик. Неопределённые значения `NA` исключаются явно,
числа валидных и парных наблюдений сохраняются.
