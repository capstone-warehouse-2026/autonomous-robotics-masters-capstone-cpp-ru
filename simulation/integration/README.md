# Мир физической интеграции

`warehouse.wbt` — самодостаточный мир Webots R2025a без внешних PROTO и текстур.
`map.pgm`/`map.yaml` — карта его статических стен и стеллажей для Nav2/AMCL.
Описание геометрии, источников odom/scan и reset:
[capstone_webots_bridge](../../ros2_ws/src/capstone_webots_bridge/README.md).

Начальная карта робота: x=−3, y=0, yaw=0. Энкодерная odom начинается с нуля.
AMCL получает известную стартовую оценку; Supervisor ground truth разрешён только
узлу оценки. Центральный стеллаж физически блокирует прямой путь к (3,0).
