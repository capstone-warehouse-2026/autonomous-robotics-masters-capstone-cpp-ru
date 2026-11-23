# Правила работы с репозиторием Capstone

## Сначала прочитайте

- [README.md](README.md) — быстрый вход, назначение групп, статус стенда.
- [docs/INTERFACES.md](docs/INTERFACES.md) — источник истины для ROS API; не заменяйте спецификацию текущим поведением фикстуры.
- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md), [docs/WORKFLOW.md](docs/WORKFLOW.md) и [.github/CODEOWNERS](.github/CODEOWNERS) — среда, ownership, review.
- [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md), [docs/GRADING.md](docs/GRADING.md), [projects/](projects/) — правила оценки, сценарии и реализация подпроектов.

## Карта кода

| Зона | Путь |
|---|---|
| G1: фильтр, scan-to-map, ROS-узел | `ros2_ws/src/capstone_localization/` |
| Симулятор Webots R2025a/TIAGo, датчики и камеры | `simulation/`, `ros2_ws/src/capstone_sim/`, `ros2_ws/src/capstone_sim_rgbd/` |
| G2 / G3 / G4 | `capstone_perception` / `capstone_planning` / `capstone_control` + `capstone_safety` в `ros2_ws/src/` |
| G5 и общая проверка | `capstone_mission`, `capstone_bringup`, `capstone_benchmark` в `ros2_ws/src/` |
| Интеграция без симулятора | [integration/g1_v1/](integration/g1_v1/README.md) |
| Подготовка данных и обучение | `ml/localization/`, `ml/perception/` |

## Перед изменением и проверкой

- Проверьте текущую ветку и `git status --short`. Не откатывайте чужие изменения и не добавляйте весь рабочий каталог в коммит. Для изменения интерфейса нужен review издателя и потребителя, ADR при изменении соглашений и контрактный тест. Не меняйте одновременно окружение и алгоритм без нужды.
- Цель: Jazzy / Ubuntu 24.04 **в Docker**. Сдаваемый код робота и метрики — C++17 (ROS-узлы: rclcpp); Python здесь для подготовки данных, обучения и разработческих фикстур. Нативные Humble-команды — исторический вспомогательный путь, не проверка курса.
- Сначала узкий тест: `make test PKG=capstone_localization` или `make test PKG=capstone_benchmark`; подставьте имя своего пакета. Из корня запустите `bash integration/g1_v1/course_smoke.sh mock` (контракт синтетики), `bash integration/g1_v1/course_smoke.sh real` (та же синтетика + C++ G1) или `bash integration/g1_v1/course_smoke.sh replay` (запись и воспроизведение). Образ dev может потребовать сборки/скачивания, но эти режимы не запускают Webots.
- `make smoke` запускает сцену Webots и проверяет `/clock`; это не проверка выполнения миссии. На одном хосте не запускайте вторую сцену поверх чужой. Не останавливайте чужие контейнеры/процессы. Для доков достаточно проверки ссылок, синтаксиса и команд; если runtime не запускали, так и пишите.
- Не коммитьте большие rosbag, архивы симулятора, логи, веса, build/install, токены или локальные файлы. Для малых фикстур указывайте происхождение и SHA-256; артефакты экспериментов отделяйте от тестов и моков. Не публикуйте артефакт от лица всей группы без явного указания.

## Инварианты интеграции

- `use_sim_time:=true` при симуляции/replay. Один источник `/clock`. При воспроизведении записанного `/clock` не добавляйте `ros2 bag play --clock`. Изолируйте DDS-граф; не соединяйте fixture/replay с активным Webots или реальным роботом.
- Сохраняйте `map → odom → base_link`, frame/stamp и владельца каждого TF. Когда SLAM публикует `map→odom`, запускайте G1 с `publish_map_tf:=false`. Карта: `nav_msgs/msg/OccupancyGrid`, frame `map`, reliable / transient-local / depth 1. Статический TF тоже transient-local; датчики подписываются с SensorDataQoS. Не полагайтесь на точное соблюдение номинальных частот в стенде.
- Mock-выходы и настоящие G1-выходы не должны работать одновременно. Для входного replay не публикуйте старые `/tf`, `/localization/pose`, `/state/odom`, команды или GT. Оригинальные `/tf_static` и записанный `/clock` нужны потребителю. Единственный штатный издатель `/cmd_vel` — G4 safety monitor.
- GT — только отдельный evaluator. Симулятор может публиковать GT в общем DDS-графе; наличие издателя не означает изоляцию. В офлайн-инференс передавайте только входной bag, без файла GT. Не подписывайте оценщик состояния, модель или контроллер на `/evaluation/ground_truth`. У пробной train_0 записи метка GT `map`, но числа в Webots world: используйте [хешированный контракт прогона](ml/localization/train0_sealed_scoring_contract.json) только в оценке; не применяйте этот частный сдвиг к другим записям. Метрики raw в общей СК первичны; стартовое выравнивание только диагностика.
- Не выдавайте фикстуру, одиночный проезд, краткий лог или fallback за зачётный A/B. Разделяйте миры/эпизоды train, validation и test, сохраняйте неуспехи, seeds и версии карты/данных/модели. Порог, fault windows и правила отбора меняются до теста, не после просмотра результата.
