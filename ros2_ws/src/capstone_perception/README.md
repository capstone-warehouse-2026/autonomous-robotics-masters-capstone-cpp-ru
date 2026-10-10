# `capstone_perception` — G2: восприятие препятствий и локальная карта

Сокращения раскрыты при первом употреблении; [полный словарь терминов](../../../docs/GLOSSARY.md).

Задание: [projects/02_perception.md](../../../projects/02_perception.md), контракты: [docs/INTERFACES.md](../../../docs/INTERFACES.md), камера: [ADR 0003](../../../docs/adr/0003-g2-rgbd-camera.md).

## Как устроен узел

```
/sensors/rgb ─┐
/sensors/depth ├─ синхронизация по метке времени ─► RgbdFrame ─► PixelClassifier ─► классы пикселей ─► GridBuilder ─► /perception/obstacles
/sensors/camera_info ┘                  (+ TF камеры)        A: GeometricClassifier                    (общий для A и B)
                                                             B: CnnClassifier ─(ошибка)─► A
```

Оба метода отвечают на один вопрос — **чем является каждый пиксель** — через общий интерфейс [`PixelClassifier`](include/capstone_perception/pixel_classifier.hpp). Всё остальное общее: синхронизация входов, TF, проверка ответа, проекция в сетку, публикация и диагностика. Поэтому сравнение A и B честное: отличается только классификация.

| Файл | Кто ведёт | Что внутри |
|---|---|---|
| [`rgbd_frame.hpp`](include/capstone_perception/rgbd_frame.hpp) | общий | `RgbdFrame`: `rgb` (`CV_8UC3`, RGB), `depth` (`CV_32FC1`, м, `NaN` — нет измерения), `camera` (fx, fy, cx, cy), `base_from_camera` (поза `camera_optical_frame` в `base_link`), `stamp` |
| [`pixel_classifier.hpp`](include/capstone_perception/pixel_classifier.hpp) | общий | интерфейс и `validate_labels()` |
| [`geometric_classifier.*`](src/geometric_classifier.cpp) | **участник 1, метод A** | сейчас заглушка: все пиксели `kUnknown` |
| [`cnn_classifier.*`](src/cnn_classifier.cpp) | **участник 2, метод B** | сейчас заглушка: без модели бросает исключение, узел переходит на A |
| [`grid_builder.*`](src/grid_builder.cpp) | общий | проекция классов в окно 10 × 10 м по 0.05 м |
| [`perception_node.*`](src/perception_node.cpp) | общий | подписки, TF, резервный переход, публикация, `/diagnostics` |

Изменения общих файлов — через PR с review второго участника.

## Контракт классификатора

```cpp
class PixelClassifier {
 public:
  virtual std::string name() const = 0;                     // "classical" или "learned"
  virtual cv::Mat classify(const RgbdFrame &frame) = 0;     // CV_8UC1 размера depth
};
enum class PixelClass : std::uint8_t { kFree = 0, kObstacle = 1, kUnknown = 255 };
```

- Значения — только 0, 1 или 255. Пиксель с `NaN` в глубине обязан быть `kUnknown`: свободное место без геометрии не объявляется. Узел проверяет это для каждого кадра (`validate_labels`).
- Если кадр нельзя классифицировать — бросить `std::runtime_error`. У метода B это включает резервный переход на A (счётчик `fallbacks` в `/diagnostics`); у метода A кадр пропускается, и устаревшая карта не публикуется.
- Свои параметры метод объявляет в конструкторе на узле с префиксом `classical.` или `learned.` и добавляет в [config/perception.yaml](config/perception.yaml).
- Классификатор не знает про ROS-топики и TF: всё нужное есть в `RgbdFrame`. Его можно тестировать на картинках без симулятора.

## Как из классов получается сетка

`GridBuilder` берёт каждый `pixel_stride`-й пиксель с классом `kFree` или `kObstacle` и конечной глубиной, переводит его в точку (`X = (u − cx)·z/fx`, `Y = (v − cy)·z/fy`, `Z = z`), затем через TF в `target_frame` и отдаёт голос клетке. Клетка — `100`, если голосов «препятствие» не меньше `grid.min_obstacle_points`, иначе `0`, если голосов «свободно» не меньше `grid.min_free_points`, иначе `−1`. Окно центрировано на роботе, `origin` кратен шагу сетки, препятствия не раздуваются.

## Запуск

Симулятор в одном терминале (`make sim` или `make sim-gui`), в другом:

```bash
make build PKG=capstone_perception
make shell
```

В контейнере (пока G1 не публикует `odom`, сетка строится в `base_link`):

```bash
ros2 launch capstone_perception perception.launch.py method:=classical target_frame:=base_link
```

Проверка: `ros2 topic hz /perception/obstacles`, `ros2 topic echo /diagnostics`, карта — в `make rviz` (дисплей «Obstacles»). Пока метод A — заглушка, вся сетка `−1`; с `method:=learned` без модели в `/diagnostics` видны переходы на классический метод.

## Тесты

```bash
make test PKG=capstone_perception
```

[test_grid_builder.cpp](test/test_grid_builder.cpp) проверяет проекцию на синтетических кадрах: клетка препятствия, `NaN` и `kUnknown` без голосов, приоритет препятствия, пороги, выравнивание окна, проверку ответа классификатора. Свои тесты метода добавлять в `test/` и в `CMakeLists.txt`.
