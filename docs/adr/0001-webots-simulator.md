# ADR 0001: Webots R2025a вместо Gazebo Harmonic

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../GLOSSARY.md).

ADR (Architecture Decision Record — запись об архитектурном решении). Статус: принято командой до недели 2, как требует [DEVELOPMENT.md](../DEVELOPMENT.md).

## Решение

Все группы используют Webots R2025a с мостом `webots_ros2` 2025.0.x (ROS (Robot Operating System — программная платформа для робототехники) 2 Jazzy). Gazebo Harmonic удалён из образов.

## Как это закреплено

- Webots устанавливается в этап `simulator` из официального архива `webots-R2025a-x86-64.tar.bz2` с проверкой SHA256 (Secure Hash Algorithm, 256-bit — алгоритм хеширования с результатом длиной 256 бит). Готового образа Cyberbotics для Ubuntu 24.04 нет: на Docker Hub есть только `R2025a-ubuntu22.04`, без ROS 2 Jazzy.
- Эксперименты и CI (Continuous Integration — непрерывная интеграция; автоматическая сборка и проверки) запускают Webots без окна внутри Xvfb (X virtual framebuffer — виртуальный дисплей X11) с программным рендерингом Mesa: изображения камер и глубины не зависят от видеокарты участника.
- `/clock` публикует `Ros2Supervisor` из `webots_ros2_driver`. Интерфейсы из [INTERFACES.md](../INTERFACES.md) не меняются. Робот и мир — [ADR 0002](0002-tiago-base-warehouse.md).
- Первая строка каждого мира — `#VRML_SIM R2025a utf8`. Миры сохраняются только из Webots R2025a, стандартные модели подключаются по адресам `EXTERNPROTO` с тегом `R2025a` и хранятся в `simulation/webots_assets` (в архиве Webots их нет), собственные PROTO и текстуры хранятся в репозитории. Стартовая проверка отклоняет мир, если Webots печатает `ERROR:`.

## Что остаётся сделать группам

- G1: модель робота и датчики, публикация `/sensors/*` с именами и QoS (Quality of Service — политики качества обслуживания при передаче сообщений) из контракта, wheel odometry (в `webots_ros2` её нет по умолчанию), ground truth и контакты для evaluator через Supervisor API (Application Programming Interface — программный интерфейс), reset сценария.
- G2: проверить кодировку камеры `webots_ros2`: контракт требует `rgb8`, а мост может выдавать `bgra8`. Выбрать конвертацию в адаптере симулятора либо изменить контракт через ADR.
- Все: определить `basicTimeStep`, `randomSeed` и число потоков физики так, чтобы частоты датчиков из контракта делились на шаг симуляции без остатка.
