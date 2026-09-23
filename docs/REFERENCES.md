# Первичные источники и технические основания

Сокращения раскрыты при первом употреблении; [полный словарь терминов](GLOSSARY.md).

Стек выбран для соответствия исходному курсу. Документацию API (Application Programming Interface — программный интерфейс) читать для Jazzy и Webots R2025a, а не автоматически переносить примеры Rolling. Версии библиотек студенты фиксируют в образах.

- [Исходный курс](https://github.com/Yazan-pyth/advanced_autonomous_mobile_robots_cpp_ru) и [правила C++ deployment](https://github.com/Yazan-pyth/advanced_autonomous_mobile_robots_cpp_ru/blob/main/docs/CPP_ML_DEPLOYMENT.md): C++ runtime, offline training и model verification.
- [Webots R2025a: руководство пользователя](https://cyberbotics.com/doc/guide/index?version=R2025a) и [справочник узлов](https://cyberbotics.com/doc/reference/index?version=R2025a): формат мира `.wbt`, PROTO, датчики, Supervisor API (Application Programming Interface — программный интерфейс).
- [Webots в Docker и без экрана](https://cyberbotics.com/doc/guide/installation-procedure?version=R2025a#run-webots-in-docker-in-headless-mode): Xvfb (X virtual framebuffer — виртуальный дисплей X11) и программный рендеринг.
- [webots_ros2](https://github.com/cyberbotics/webots_ros2) и [учебник ROS (Robot Operating System — программная платформа для робототехники) 2 Jazzy по Webots](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html): мост ROS 2–Webots, `Ros2Supervisor` и `/clock`. Версия 2025.0.x соответствует Webots R2025a.
- [Nav2 (Navigation 2 — стек навигации для второй версии платформы Robot Operating System) Collision Monitor, Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/): архитектурный пример независимого защитного уровня; проект требует собственного обоснования safety envelope.
- [ONNX (Open Neural Network Exchange — открытый формат обмена моделями нейронных сетей) Runtime C++](https://onnxruntime.ai/docs/get-started/with-cpp.html): C++ inference API.
- [PPO (Proximal Policy Optimization — алгоритм оптимизации политики с ограничением величины её обновления), авторская статья](https://arxiv.org/abs/1707.06347): алгоритм для G4.
- [DQN (Deep Q-Network — глубокая нейронная сеть для оценки ценности действий), авторская статья](https://www.nature.com/articles/nature14236): алгоритм для G5.

Эти источники обосновывают инструменты и алгоритмы, а распределение команд, сценарии, лимиты, grading и объём экспериментов являются требованиями данного учебного задания. Значения latency и safety margins не являются гарантией библиотек или сертификатом безопасности.
