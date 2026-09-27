# Разработка и Docker

Сокращения раскрыты при первом употреблении; [полный словарь терминов](GLOSSARY.md).

## Что уже существует

- Три Docker build targets: `simulator` (Webots R2025a + `webots_ros2_driver`), `dev` (компиляторы, ccache, gdb и все зависимости из `package.xml` через rosdep) и `autonomy` (`dev` + собранный и протестированный workspace C++ ROS (Robot Operating System — программная платформа для робототехники) для CI и сдачи).
- Заготовки пакетов всех групп в `ros2_ws/src`: узел с параметром `method: classical|learned`, общая библиотека узла и gtest-тесты. Владельцы — в `.github/CODEOWNERS`.
- Мир `simulation/worlds/warehouse.wbt`: склад 20 × 20 м со стеллажами, ящиками, поддонами, шестью станциями и роботом TIAGo Base — круглой дифференциальной платформой PAL Robotics ([ADR 0002](adr/0002-tiago-base-warehouse.md)). LiDAR, RGB-D камеры и ground truth adapter пока нет.
- `ros2_ws/src/capstone_sim`: C++ плагин `webots_ros2_driver` `DiffDrive`, который переводит `/cmd_vel` (`TwistStamped`) в скорости двух колёс робота и останавливает их без свежей команды 300 мс. Моторы и геометрия колёс задаются в `resource/tiago_base.urdf`. Собирается в этапе `simulator-build` и копируется в образ симулятора.
- `simulation/launch/warehouse.launch.py`: запускает Webots, `Ros2Supervisor` (публикует `/clock`) и драйвер робота. Любая строка `ERROR:` в выводе Webots (ошибка разбора мира, неизвестный узел, ненайденный PROTO или текстура), остановка Webots или драйвера завершают симулятор с ненулевым кодом. `SIM_WORLD=другой.wbt` выбирает другой мир из `simulation/worlds`.
- `capstone_bringup/clock_probe`: C++ подписчик, проверяющий несколько строго возрастающих значений `/clock` и завершающийся с ненулевым кодом при wall-clock timeout.
- Unit test на frozen/regressing/invalid clock; в GitHub Actions — shellcheck скриптов и полный `make setup` на Ubuntu 22.04 и 24.04.

Это инфраструктурная проверка DDS (Data Distribution Service — стандарт обмена данными распределённых систем) между двумя контейнерами. Студенты реализуют robot/world assets, navigation, datasets/models и final benchmark. CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки) starter не проверяет качество несуществующих алгоритмов.

## Команды

Компьютер готовит `scripts/setup_host.sh` (git, make, Docker Engine через get.docker.com, группа `docker`), проверяет `make doctor`. Первый `make setup` скачивает пакеты ROS и архив Webots (150 МБ, проверяется по SHA256 (Secure Hash Algorithm, 256-bit — алгоритм хеширования с результатом длиной 256 бит)) и собирает образы; GPU (Graphics Processing Unit — графический процессор) и GUI (Graphical User Interface — графический интерфейс пользователя) для него не нужны. Полный список — `make`.

| Команда | Что происходит |
|---|---|
| `make build [PKG=имя]` | `colcon build --symlink-install` в контейнере `dev`; с `PKG` — пакет и его зависимости. ccache ускоряет повторную сборку |
| `make test [PKG=имя]` | `colcon test` и `colcon test-result --verbose` |
| `make shell` | интерактивный bash в `dev` с настроенными ROS и workspace |
| `make smoke` | сборка `simulator` и `autonomy`, затем `docker compose up` — та же проверка, что в CI |
| `make sim` | симулятор без окна; мир берётся из рабочей копии `simulation/`, пересборка образа не нужна |
| `make sim-gui` | симулятор с окном Webots (`scripts/sim_gui.sh`) |
| `make images` | принудительно пересобрать все образы |
| `make down`, `make clean` | остановить контейнеры; удалить `ros2_ws/build`, `install`, `log` |

Ожидаемый итог `make smoke` — `PASS: simulation clock advanced`. Сломанная сцена даёт в логе симулятора `FAIL: simulation did not start cleanly: ...`, exit code 1 симулятора и ненулевой итог всей команды.

### Контейнер разработки

Сервис `dev` в `compose.yml` монтирует весь репозиторий в `/workspace` и работает от UID/GID (User/Group Identifier — числовые идентификаторы пользователя и группы) пользователя компьютера, поэтому `ros2_ws/build`, `install` и `log` принадлежат ему и остаются между запусками. `HOME` и кэш ccache лежат в `.cache/` репозитория (не попадают в Git). Изменение кода не требует пересборки образа.

`scripts/dev_image.sh` перед каждым `make build/test/shell` сравнивает хеш `docker/Dockerfile`, `docker/dev_env.sh` и всех `package.xml` с записанным и при расхождении пересобирает `dev`. В образе rosdep ставит всё, что перечислено в `<depend>` пакетов; слой зависимостей строится только из `package.xml`, поэтому правка кода его не инвалидирует. Если зависимости нет в rosdep (например, ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime), её добавляет владелец инфраструктуры в `docker/Dockerfile`.

Пакеты собраны с `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`; `compile_commands.json` в `ros2_ws/build/<пакет>` содержит пути контейнера (`/workspace/...`).

### Окно Webots

`make sim-gui` передаёт в контейнер X11-сокет и копию cookie дисплея (`xauth`), работает на Xorg и на Wayland через XWayland. Если есть `/dev/dri`, Webots использует видеокарту (Intel/AMD, открытый драйвер Mesa); иначе рисует программно — медленнее, но сцена та же. Проприетарный драйвер NVIDIA в контейнер не пробрасывается. Мир монтируется из рабочей копии с правом записи: сохранённый в окне `.wbt` сразу появляется в Git. Сохранять миры только из этого Webots R2025a.

### Мир и модели Webots

Стандартные модели Webots (TIAGo Base, ящики, поддоны, покрытия) не входят в архив Webots: сам Webots загружает их с GitHub по адресам `EXTERNPROTO` версии R2025a, и на части сетей эта загрузка обрывается. Поэтому нужные миру файлы лежат в репозитории, в [simulation/webots_assets](../simulation/webots_assets/README.md) (17 МБ, пути как в репозитории Webots). При сборке образа они копируются в кэш Webots `/opt/webots-cache`, затем `docker/check_webots_worlds.sh` загружает каждый мир **без сети** и останавливает сборку, если чего-то не хватает. Ни сборка, ни запуск ничего не скачивают, у всех участников и в CI одинаковые файлы.

Правила редактирования мира (`make sim-gui`, файл сохраняется прямо в рабочую копию):

- перед сохранением вернуть симуляцию в начало (**Reset Simulation**), иначе в файл попадут строки `hidden ...` с текущими скоростями и положениями;
- новые стандартные объекты добавлять через **Add node** из Webots R2025a — адрес `EXTERNPROTO` будет с тегом `R2025a`; затем выполнить `scripts/update_webots_assets.sh` (один раз скачивает архив ресурсов Webots 693 МБ в `.cache/` и добавляет нужные файлы в `simulation/webots_assets`) и закоммитить результат; собственные PROTO класть в `simulation/protos` и подключать относительным путём;
- комментарии `#` Webots при сохранении удаляет — пояснения к миру держать в [ADR 0002](adr/0002-tiago-base-warehouse.md);
- после изменения мира `make smoke` проверяет, что он загружается без ошибок.

Проверить управление: в одном терминале `make sim`, в другом `make shell` и

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"
```

Робот едет, пока идут команды, и останавливается через 0.3 с после `Ctrl+C`.

### Без Docker

Локальная ROS 2 (Robot Operating System 2 — вторая версия программной платформы для робототехники) среда возможна только на Ubuntu 24.04 (ROS 2 Jazzy не выпускается для 22.04) и не поддерживается командами `make`. `clock_probe` сам по себе завершится ошибкой без живого `/clock`; это ожидаемая проверка, не standalone navigation demo.

## Контейнерная архитектура

Compose использует общую bridge network, один `ROS_DOMAIN_ID=42`, middleware CycloneDDS и UDP (User Datagram Protocol — протокол передачи пользовательских дейтаграмм) discovery. `ROS_LOCALHOST_ONLY=0`; host networking, privileged mode и host Docker socket не нужны. Webots и мост `webots_ros2` работают внутри simulator container (мост подключается к Webots локально через общую память, поэтому `shm_size: 1gb`), C++ probe внутри autonomy. Если локальный Docker/network фильтрует multicast, зафиксировать CycloneDDS peer configuration и проверить её на обеих машинах; не считать успешный native запуск доказательством двухконтейнерной связи.

Build base `ros:jazzy-ros-base` обновляется upstream. Starter не обещает bit-for-bit repeatability этого тега. Для итоговой сдачи сохранить image IDs (Identifiers — идентификаторы)/digests, список пакетов и base digest; использовать один замороженный образ для всей серии. ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime/LibTorch и solver пока не установлены — каждая группа добавляет совместимые фиксированные версии и license metadata.

Симулятор работает от непривилегированного пользователя `ubuntu`. По умолчанию `docker/start_simulator.sh` задаёт `WEBOTS_OFFSCREEN=1`: Webots запускается внутри Xvfb (X virtual framebuffer — виртуальный дисплей X11) с программным рендерингом Mesa, поэтому изображения камер не зависят от видеокарты компьютера. Предупреждение Webots `System below the minimal requirements` в этом режиме ожидаемо. `SIM_GUI=1` открывает окно Webots; его выставляет `make sim-gui` вместе с `compose.gui.yml` и, при наличии `/dev/dri`, `compose.dri.yml`. В `autonomy` probe имеет 45-секундный steady-clock deadline, чтобы отсутствие `/clock` не приводило к зависанию CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки). При создании реального bringup заменить smoke CMD (Command — инструкция Dockerfile, задающая команду контейнера по умолчанию) на запуск всех компонентов, сохранив smoke как отдельный тест.

## Требования к финальным контейнерам

1. Clean clone → documented build → один запуск mission без ручной правки файлов контейнера.
2. Headless режим обязателен; GUI/RViz (ROS Visualization — средство визуализации данных робототехнической платформы) — optional profile. CPU (Central Processing Unit — центральный процессор) mode обязателен; GPU compose override отдельный.
3. Sensor/command bridges, TF (Transform library — библиотека преобразований между системами координат), `/clock`, seed/reset и readiness тестируются между контейнерами.
4. Models/configs read-only; results в mounted output directory. No hardcoded workstation paths.
5. Dependency versions, model SHA256 (Secure Hash Algorithm, 256-bit — алгоритм хеширования с результатом длиной 256 бит), image digest и commit находятся в run manifest.
6. Один контейнер вместо двух разрешён с теми же критериями; выбор не даёт дополнительных баллов сам по себе.

Альтернативный симулятор разрешён после общего решения до недели 2: требуется ROS adapter, датчики, contact/GT (Ground Truth — эталонные данные для обучения или оценки) для evaluator, deterministic seeding насколько поддерживается, reset и Docker/headless запуск. Поздняя замена не отменяет interfaces/benchmark.
