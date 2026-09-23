# Работа команды, Git и календарь

Сокращения раскрыты при первом употреблении; [полный словарь терминов](GLOSSARY.md).

## Ответственность подгрупп

| Подгруппа | Основная ответственность | Интеграционная обязанность |
|---|---|---|
| G1 | Локализация, карта, симулятор и датчики | Согласовать состояние робота, системы координат, время и сброс |
| G2 | Геометрическое и обучаемое восприятие | Передать карту препятствий планировщику и контроллеру |
| G3 | Классическое и обучаемое планирование | Согласовать путь, отмену задачи и перепланирование |
| G4 | Классическое и обучаемое управление, безопасность | Обеспечить защитную остановку и обратную связь о движении |
| G5 | Миссии, диспетчеризация, общая сборка и эксперименты | Согласовать запуск, журналы и расчёт общих метрик со всеми группами |

Обязанности распределяются внутри каждой группы. Каждый участник должен понимать оба метода своего подпроекта и участвовать в интеграции. Координация интеграции передаётся между группами раз в две недели; запуск системы и диагностика должны быть доступны всей команде.

## Milestones на 14 недель

| Неделя | Результат и критерий готовности |
|---|---|
| 1 | Команды, research questions, literature review, clone/build starter, учёт доступных CPU (Central Processing Unit — центральный процессор)/GPU (Graphics Processing Unit — графический процессор) |
| 2 | Freeze интерфейсов v1, simulator choice, sensors/limits, split и scenario manifests, budget на обучение |
| 3–4 | Robot/sensors + все classical stubs заменены рабочими методами; первая C0 доставка; CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки) и replay |
| 5–6 | Устойчивый C0, train data, correctness tests, baseline metrics; никаких ожиданий готовности чужой модели |
| 7–8 | Все пять learned variants работают из C++; model cards и первые isolated comparisons |
| 9–10 | C1…C6, fault tests, полный reset, profiler; freeze dataset/model selection по validation |
| 11 | Code/config freeze, final benchmark rehearsal, capacity check; smoke не считается итоговым экспериментом |
| 12 | Final 560 system runs, isolated comparisons и training-seed evaluation; только documented reruns |
| 13 | Analysis, CI (Confidence Interval — доверительный интервал)/effect sizes, failure cases, five reports + integration draft, воспроизведение другой группой |
| 14 | Release, демонстрация C0/C6, fault demo и индивидуальная защита |

При 20 часах проекта в неделю на подгруппу суммарная capacity порядка 1400 group-hours; конкретное расписание преподаватель адаптирует до старта. Время GPU и длительность прогонов резервируются заранее. Дополнительные архитектуры/симуляторы допустимы только после выполнения core requirements.

## Git-процесс

- Один общий репозиторий с пакетами `capstone_localization`, `capstone_perception`, `capstone_planning`, `capstone_control`, `capstone_safety`, `capstone_mission`, `capstone_benchmark`. Все пакеты уже созданы как заготовки: узел с параметром `method: classical|learned` и тесты; группы наполняют их кодом. Владельцы пакетов записаны в [.github/CODEOWNERS](../.github/CODEOWNERS).
- Branch `gN/issue-description`, короткие PR (Pull Request — запрос на включение изменений в репозиторий) в `main`. Один review внутри группы; для интерфейса — review группы-потребителя. В PR: задача, изменения, проверки, reproduction command и связанные issues.
- Issues имеют group, milestone и owner. Чисто документальная правка не требует симуляции, но изменение интерфейса требует contract test.
- Не коммитить bags/checkpoints/secrets/build products. Малые fixtures — с происхождением и лицензией; модели — release assets с SHA256 (Secure Hash Algorithm, 256-bit — алгоритм хеширования с результатом длиной 256 бит).
- Теги: `v0.1-contracts`, `v0.2-classical`, `v0.3-learned`, `v1.0-submission`. Starter публикуется как `v0.1.0-assignment`, чтобы не путать его с результатами студентов.
- `main` защищён: изменения только через PR, обязательны зелёный CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки) на Ubuntu 22.04 и 24.04 и одобрение владельца изменённого кода по CODEOWNERS. Владелец репозитория включает это один раз по [GITHUB_SETUP.md](GITHUB_SETUP.md).

## Совместная разработка без блокировок

Каждая группа предоставляет небольшой bag/input fixture и reference outputs. До live integration consumers работают с replay. G1 подготавливает simulator adapter, G4 — safety, G5 — bringup/runner; остальные поставляют свои launch/config и зависимости. На integration meeting показывать один воспроизводимый дефект и его regression check, а не только slides.

Изменение контракта: issue → запись причины/альтернатив/миграции в `docs/adr/` → review producer+consumer → обновление tests → merge. Новый learned method не должен ломать classical path.
