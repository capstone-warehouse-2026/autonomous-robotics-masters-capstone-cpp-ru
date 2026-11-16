# Обучение метода B (G4): PPO

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../docs/GLOSSARY.md).

Офлайн-инструменты обучаемого метода B на Python: среда, обучение, оценка и экспорт. Работа политики в роботе — на C++ (`PpoController` в [capstone_control](../../ros2_ws/src/capstone_control)), через тот же интерфейс `Controller` и тот же `CommandLimiter`, что у MPC (Model Predictive Control — управление с прогнозирующей моделью) и pure pursuit ([ADR 0004](../../docs/adr/0004-g4-follow-path.md)). Задание: [projects/04_control.md](../../projects/04_control.md).

PPO (Proximal Policy Optimization — алгоритм оптимизации политики с ограничением величины её обновления) выдаёт целевые `(v, ω)` по тем же входам, что и MPC: активный участок пути от `PathTracker`, препятствия в `base_link` и измеренная скорость. Ground truth и будущие положения препятствий политика не получает.

## Файлы

| Файл | Что делает |
|---|---|
| [control_core.py](control_core.py) | Порт на Python `PathTracker` и `CommandLimiter` из `capstone_control`. Готов, не TODO: проверен против C++ построчно |
| [spec.py](spec.py) | **Контракт** наблюдения и действия. C++ (`ppo_observation.hpp`) повторяет его буквально |
| [env.py](env.py) | Среда `gymnasium`: процедурные сцены, модель робота, domain randomization, reward |
| [train.py](train.py) | PPO из Stable-Baselines3, 3 training seeds, выбор модели по validation |
| [evaluate.py](evaluate.py) | Оценка на `test` и `ood`, одна строка CSV на эпизод |
| [export_onnx.py](export_onnx.py) | ONNX-модель, манифест и golden-данные в пакет `capstone_control` |
| `test_*.py` | Спецификация на pytest. `test_spec.py` проверяет те же числа, что `test_ppo_observation.cpp`; `test_control_core.py` — те же случаи, что тесты C++ коллеги |

Если в C++ поменяется `PathTracker` или `CommandLimiter`, `control_core.py` и `test_control_core.py` меняются в том же PR: иначе политика учится не на том роботе.

## Наблюдение и действие

61 число float32 ([spec.py](spec.py)):

| Индексы | Содержимое | Нормализация |
|---|---|---|
| 0–19 | 10 точек `PathTracker::point_ahead` через каждые 0.2 м от проекции робота, в `base_link` | `/ 2 м` |
| 20 | оставшаяся длина пути | `min(·, 5 м) / 5 м` |
| 21–22 | `cos`, `sin` угла цели относительно робота | — |
| 23–58 | 36 секторов по 10° от −180°: расстояние до ближайшей точки препятствия в секторе | `min(·, 3 м) / 3 м` |
| 59–60 | измеренные `v`, `ω` | `/ 0.5 м/с`, `/ 1.0 рад/с` |

Действие — 2 числа в `[-1, 1]` → цель `v ∈ [0, 0.5]` м/с, `ω ∈ [-1, 1]` рад/с. Дальше команда проходит **общий `CommandLimiter`** ADR 0004 (разгон 0.5 м/с², торможение 1.0 м/с², угловое 2.0 рад/с²) — в среде так же, как в узле. Поэтому оба метода физически не могут выдать разные по допустимости команды.

## Среда

Не Webots, а процедурная 2D-сцена: обучению нужны миллионы шагов. Модель робота — точная модель unicycle по дуге, как в `MpcController::rollout`, такт 50 мс (20 Гц, как узел).

- **Сцены** по сиду: путь из 3–5 отрезков с поворотами, точки через 0.05–0.1 м (как у G3), иногда с поворотом в конце; коробки вдоль пути (путь от планировщика свободен), иногда узкий проход, иногда движущееся препятствие поперёк пути.
- **Разбиение**: `train` 0–999, `val` 1000–1099, `test` 2000–2019, `ood` 3000–3019. Среда отказывается работать с сидом не своего разбиения.
- **Domain randomization** (только обучение): коэффициенты исполнения `v` и `ω` 0.8–1.2 (это и есть model mismatch ±20 % из теста), задержка команды 0–2 такта, шум одометрии, шум и пропуски точек препятствий. Ablation — `--no-randomization`.
- **Reward**: прогресс вдоль пути, штрафы за поперечное отклонение, рывки, близость к препятствиям (с 0.45 м, как `d_safe` у MPC) и время; на последних 0.5 м — potential-based shaping к позе цели (расстояние и угол); столкновение −50, уход с пути дальше 1 м −50 (эпизод заканчивается), цель +50.
- **Успех** — тот же, что у `GoalChecker` в узле: ≤ 0.25 м, ≤ 0.2 рад, `|v| < 0.05`, `|ω| < 0.1` по измеренной скорости.

## Команды

Через Docker (цели `g4-*` добавляет `g4_infra.patch`, см. ниже):

```bash
make g4-ml-test                                     # pytest: control_core, spec, env
make g4-train                                       # сиды 0 1 2, domain randomization
make g4-train G4_SEEDS=0 TRAIN_ARGS=--no-randomization   # ablation
make g4-eval                                        # test; EVAL_ARGS="--split ood" или "--mismatch 0.8"
make g4-model CHECKPOINT=data/g4/models/seed_<лучший по val>/best_model.zip
make test PKG=capstone_control                      # golden-тест C++ против Python
```

Без Docker — в venv на хосте:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r ml/control/requirements.txt
cd ml/control && python3 -m pytest -q
python3 train.py --seed 0
```

Модели и логи пишутся в `data/g4/` (в Git не попадает). В пакет попадают только `models/ppo_policy.onnx`, `models/ppo_policy.json` и `test/data/ppo_golden.txt`; `*.onnx` разрешён в `.gitignore` только для `capstone_control/models`, как у G2.

## Что учесть

- **Скорость среды.** Наивная реализация на Python — несколько сотен шагов в секунду. Для 3 × 3 млн шагов геометрию (секторы, просвет до препятствий) нужно векторизовать в numpy, а среды запускать параллельно (`SubprocVecEnv`, `--envs`). Время обучения каждого seed — в манифест.
- **Одинаковые сцены для A и B.** Изолированное сравнение требует одинаковых входов: либо MPC в этой же среде (обёртка `MpcController` для Python), либо оба метода в Webots replay через `capstone_benchmark`. Решить в группе до заморозки экспериментов.
- **Выбор модели** — только по `val`. Все три seed показываются в отчёте; `test` и `ood` не читаются при обучении.
