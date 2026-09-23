# Словарь сокращений и обозначений

В документах сокращения раскрываются при первом употреблении: английское полное название и русское объяснение. Здесь собраны также обозначения из кода, конфигураций и схем. В командах, именах файлов, пакетах и типах сообщений сохраняется точное программное написание.

Например: **KF (Kalman Filter — фильтр Калмана)** и **EKF (Extended Kalman Filter — расширенный фильтр Калмана)**. Порядок слов в стандартном английском названии — Kalman Filter, поэтому используется KF, а не FK.

## Алгоритмы, инструменты и метрики

| Сокращение | Полное название | Объяснение |
|---|---|---|
| 2D | Two-Dimensional | двумерный |
| A* | A-star | алгоритм поиска пути с оценкой уже пройденной стоимости и эвристикой оставшегося пути |
| ADR | Architecture Decision Record | запись об архитектурном решении |
| API | Application Programming Interface | программный интерфейс |
| ATE | Absolute Trajectory Error | абсолютная ошибка траектории |
| BT | Behavior Tree | дерево поведения |
| CMD | Command | инструкция Dockerfile, задающая команду контейнера по умолчанию |
| CNN | Convolutional Neural Network | свёрточная нейронная сеть |
| CPU | Central Processing Unit | центральный процессор |
| CSV | Comma-Separated Values | табличный текстовый формат со значениями, разделёнными запятыми |
| DDS | Data Distribution Service | стандарт обмена данными распределённых систем |
| DL | Deep Learning | глубокое обучение |
| DQN | Deep Q-Network | глубокая нейронная сеть для оценки ценности действий |
| EKF | Extended Kalman Filter | расширенный фильтр Калмана |
| FOV | Field of View | поле зрения датчика |
| GPU | Graphics Processing Unit | графический процессор |
| GT | Ground Truth | эталонные данные для обучения или оценки |
| GUI | Graphical User Interface | графический интерфейс пользователя |
| Hz | Hertz | герц, число циклов в секунду |
| ICP | Iterative Closest Point | итеративный алгоритм ближайших точек для совмещения облаков точек |
| ID | Identifier | идентификатор |
| IDs | Identifiers | идентификаторы |
| IMU | Inertial Measurement Unit | инерциальный измерительный модуль |
| Inf | Infinity | специальное значение бесконечности |
| IoU | Intersection over Union | отношение площади пересечения к площади объединения |
| KF | Kalman Filter | фильтр Калмана; стандартное английское сокращение — KF, а не FK |
| LiDAR | Light Detection and Ranging | измерение расстояний с помощью света; лазерный дальномер |
| MARL | Multi-Agent Reinforcement Learning | многоагентное обучение с подкреплением |
| ML | Machine Learning | машинное обучение |
| MLP | Multilayer Perceptron | многослойный перцептрон |
| MPC | Model Predictive Control | управление с прогнозирующей моделью |
| ms | Millisecond | миллисекунда, одна тысячная секунды |
| NA | Not Available | значение отсутствует или не определено для данного случая |
| NaN | Not a Number | специальное значение «не число» |
| Nav2 | Navigation 2 | стек навигации для второй версии платформы Robot Operating System |
| NEES | Normalized Estimation Error Squared | нормированный квадрат ошибки оценивания |
| NIS | Normalized Innovation Squared | нормированный квадрат невязки измерения |
| ONNX | Open Neural Network Exchange | открытый формат обмена моделями нейронных сетей |
| OOD | Out of Distribution | данные или условия вне обучающего распределения |
| OpenCV | Open Source Computer Vision Library | открытая библиотека компьютерного зрения |
| opset | Operator Set | набор и версия операций модели |
| p50 | 50th percentile | 50-й процентиль, медиана |
| p95 | 95th percentile | 95-й процентиль |
| p99 | 99th percentile | 99-й процентиль |
| PCL | Point Cloud Library | библиотека обработки облаков точек |
| PPO | Proximal Policy Optimization | алгоритм оптимизации политики с ограничением величины её обновления |
| PR | Pull Request | запрос на включение изменений в репозиторий |
| QoS | Quality of Service | политики качества обслуживания при передаче сообщений |
| RANSAC | Random Sample Consensus | оценивание модели по согласованности случайных выборок |
| RGB | Red, Green, Blue | красный, зелёный и синий цветовые каналы |
| RGB-D | Red, Green, Blue and Depth | цветное изображение и карта глубины |
| RL | Reinforcement Learning | обучение с подкреплением |
| RMSE | Root Mean Square Error | среднеквадратическая ошибка |
| ROI | Region of Interest | область интереса, в которой проводится оценка |
| ROS | Robot Operating System | программная платформа для робототехники |
| ROS 2 | Robot Operating System 2 | вторая версия программной платформы для робототехники |
| RPE | Relative Pose Error | относительная ошибка положения и ориентации |
| RSS | Resident Set Size | объём физической памяти, занятой процессом |
| RViz | ROS Visualization | средство визуализации данных робототехнической платформы |
| SE(2) | Special Euclidean Group in Two Dimensions | группа перемещений и поворотов твёрдого тела на плоскости |
| SHA | Secure Hash Algorithm | семейство алгоритмов хеширования; здесь обозначение хеша коммита |
| SHA256 | Secure Hash Algorithm, 256-bit | алгоритм хеширования с результатом длиной 256 бит |
| SI | Système international d’unités | Международная система единиц |
| SLAM | Simultaneous Localization and Mapping | одновременная локализация и построение карты |
| SPL | Success weighted by Path Length | успешность навигации, взвешенная по эффективности длины пути |
| TF | Transform library | библиотека преобразований между системами координат |
| UDP | User Datagram Protocol | протокол передачи пользовательских дейтаграмм |
| URL | Uniform Resource Locator | унифицированный указатель ресурса; адрес ресурса |
| XML | Extensible Markup Language | расширяемый язык разметки |
| CI | Confidence Interval | доверительный интервал |
| CI | Continuous Integration | непрерывная интеграция; автоматическая сборка и проверки |

**CI имеет два значения.** В таблицах результатов и выражении «95% CI» это Confidence Interval (доверительный интервал). В сборке, проверках и GitHub Actions — Continuous Integration (непрерывная интеграция). В тексте оба значения подписаны явно.

## Обозначения групп, конфигураций и формул

| Обозначение | Значение |
|---|---|
| G1…G5 | Group 1…5 — пять учебных подгрупп |
| S1…S4 | Scenario 1…4 — четыре семейства сценариев |
| C0…C6, C7 | Configuration 0…6 и необязательная Configuration 7 — конфигурации интегрированной системы |
| A / B | Классический / обучаемый вариант метода |
| SE(2) | Special Euclidean Group in Two Dimensions — положение и ориентация на плоскости; число 2 обозначает размерность пространства |
| v / ω | Линейная / угловая скорость робота |
| n | Число эпизодов или наблюдений; единица анализа указывается в отчёте |
| S, L_shortest, L_actual | В формуле эффективности пути: индикатор успеха, длина кратчайшего пути, фактически пройденная длина |
| S_group, S_defence, I_system, I_individual | В формуле оценки: баллы группы, личной защиты, общей интеграции и личной интеграционной работы |
| Гц, с, м, рад, мс | Герц, секунда, метр, радиан, миллисекунда |
| м/с, м/с², рад/с, рад/с² | Единицы линейной скорости, линейного ускорения, угловой скорости и углового ускорения |
| p50 / p95 / p99 | Процентили: соответственно 50%, 95% и 99% наблюдений не превышают указанное значение |

## Сокращения внутри программных имён

| Имя или часть имени | Расшифровка и назначение |
|---|---|
| rclcpp | ROS Client Library for C++ — клиентская библиотека платформы для языка C++ |
| webots_ros2 | Интеграция Robot Operating System 2 с симулятором Webots |
| .wbt, PROTO | Файл мира Webots; PROTO — параметризуемое описание модели или объекта Webots |
| Xvfb | X virtual framebuffer — виртуальный дисплей X11 для запуска Webots без экрана |
| CycloneDDS | Реализация стандарта Data Distribution Service (служба распределения данных) |
| RMW_IMPLEMENTATION | ROS Middleware Implementation — выбор реализации промежуточного слоя связи |
| ROS_DOMAIN_ID | Robot Operating System Domain Identifier — идентификатор домена связи |
| ROS_LOCALHOST_ONLY | Robot Operating System Localhost Only — ограничение связи локальным компьютером |
| msg / msgs, srv | Message / messages, service — сообщение / сообщения, сервис |
| nav, odom, cmd_vel | Navigation, odometry, commanded velocity — навигация, одометрия, заданная скорость |
| rgb8 | Red, Green, Blue, 8 bits per channel — три цветовых канала по восемь бит |
| 32FC1 | 32-bit Floating-point, 1 Channel — один канал 32-битных чисел с плавающей точкой; для глубины значения в метрах |
| sec / nanosec / ns | Second / nanosecond — секунда / наносекунда |
| cpu, gpu, ram_gb | Central Processing Unit, Graphics Processing Unit, Random Access Memory in Gigabytes — процессоры и объём оперативной памяти в гигабайтах |
| JSON, YAML | JavaScript Object Notation; YAML Ain’t Markup Language — форматы структурированных данных и конфигураций |
| UTC | Coordinated Universal Time — всемирное координированное время |
| TO_FILL | To fill — поле, которое необходимо заполнить реальными данными эксперимента |
| PASS / FAIL | Passed / failed — проверка пройдена / не пройдена |
| SIM, SAFE, EVAL, LR | В диаграмме: simulation, safety, evaluation, left-to-right — симулятор, безопасность, оценивание, направление слева направо |
| RelWithDebInfo | Release with Debug Information — оптимизированная сборка с отладочной информацией |
| CI в имени файла ci.yml | Continuous Integration — непрерывная интеграция |
| .cpp, .hpp | Файл исходного кода / заголовочный файл на C++ |

## Названия, которые не нужно искусственно расшифровывать

**C++** — название языка, а C++17 — его стандарт 2017 года. **CMake** — имя системы настройки сборки, **CTest** — её инструмент тестирования. **Git**, **Docker**, **Webots**, **Mesa**, **Eigen**, **ament**, **colcon**, **LibTorch** и **GitHub** — имена инструментов и библиотек, а не сокращения алгоритмов. **MIT** в лицензии отсылает к Massachusetts Institute of Technology (Массачусетский технологический институт); текст лицензии сохраняется без изменений.
