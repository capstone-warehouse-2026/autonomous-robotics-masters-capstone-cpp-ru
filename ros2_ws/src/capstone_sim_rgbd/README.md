# `capstone_sim_rgbd` — G2: RGB-D камера в симуляторе

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Плагин `webots_ros2_driver` `capstone_sim_rgbd::RgbdCamera`, работает в контейнере симулятора вместе с драйвером робота. Камера — Orbbec Astra из стандартных устройств Webots R2025a в `bodySlot` TIAGo Base ([ADR 0003](../../../docs/adr/0003-g2-rgbd-camera.md)).

| Выход | Тип | Что внутри |
|---|---|---|
| `/sensors/rgb` | `sensor_msgs/Image`, `rgb8`, 640×480 | цветное изображение (Webots отдаёт BGRA, плагин переводит в RGB) |
| `/sensors/depth` | `sensor_msgs/Image`, `32FC1`, метры | планарная глубина, **пересчитанная в цветную камеру** (пиксель глубины соответствует тому же пикселю RGB); `NaN` — нет измерения: ближе 0.6 м, дальше 8 м, тень от сдвига камер |
| `/sensors/camera_info` | `sensor_msgs/CameraInfo` | общая модель для RGB и глубины: fx = fy = 558.9, cx = 319.5, cy = 239.5, без дисторсии |
| `/tf_static` | `base_link → camera_link → camera_optical_frame` | `camera_link` — цветная камера (x вперёд), `camera_optical_frame` — z вперёд, x вправо, y вниз |

Все три сообщения кадра имеют одну метку времени — время симуляции снимка; частота 10 Гц симуляционного времени, QoS (Quality of Service — политики качества обслуживания при передаче сообщений) sensor data.

## Настройка

Параметры задаются в [tiago_base.urdf](../capstone_sim/resource/tiago_base.urdf): имена устройств (`colorCamera`, `depthCamera`), `updateRate` (1000 / частота должна делиться на `basicTimeStep` мира), положение камеры на роботе `mountTranslation` и `mountRotation` (roll pitch yaw, рад). Положение должно совпадать с позой `Astra` в `bodySlot` [warehouse.wbt](../../../simulation/worlds/warehouse.wbt): сейчас 0.1 0 0.805 и наклон вниз 0.4 рад, камера на высоте 0.92 м над полом.

## Проверка

```bash
make test PKG=capstone_sim_rgbd
make sim
```

Во втором терминале, в `make shell`:

```bash
ros2 topic hz /sensors/rgb
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```
