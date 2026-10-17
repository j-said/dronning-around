# Лабораторна робота

Середовище розгорнуте в Docker, бо ROS 2 Jazzy потребує Ubuntu 24.04.

| Компонент      | Версія                                                                       |
| -------------- | ---------------------------------------------------------------------------- |
| Образ          | Ubuntu 24.04.4 LTS (`osrf/ros:jazzy-desktop`), тег `fpv-lab2:jazzy-harmonic` |
| ROS 2          | Jazzy                                                                        |
| Gazebo Sim     | Harmonic (gz-sim 8.15.0, пакунки `ros-jazzy-gz-*-vendor`)                    |
| ArduPilot SITL | гілка `master`, зібраний з DDS у `/opt/ardu_ws`                              |
| Python         | 3.12.3                                                                       |
| Mesa           | 25.2.8 (штатна з Ubuntu 24.04)                                               |

## Структура

```
Makefile                   точка входу: `make help`
scripts/lib.sh             бібліотека bash-функцій, які викликає make
docker/Dockerfile          образ (GPU через /dev/dri)
docker/ros2_gz_jazzy.repos репозиторії робочого простору ArduPilot для Jazzy
docker-compose.yml         опис контейнера
fpv_labs/src/              код польотів (звичайні модулі Python, без ROS-пакета і colcon)
  drone.py                   єдиний вузол: зв'язок з ArduPilot і Gazebo, виконання місії по кроках
  links/                     зовнішні канали, що передаються в Drone ззовні
    mavlink.py                 MAVLink: місії, параметри (чого немає в DDS-інтерфейсі ArduPilot)
  handlers.py                бібліотека обробників кроків (arm, зліт, політ до точки, посадка, місія...)
  missions/                  фази місій (prepare, takeoff, wait, fly_to, landing, auto_mission) і chain
  tools/                     офлайн-інструменти: analysis.py (таблиця 7.4 і графіки з .csv і .tlog QGroundControl)
  lab_2/                     місії лабораторної 2: ланцюжок фаз із параметрами варіанта 21
  lab_7/                     лабораторна 7: flight.py (місія AUTO), analysis.py (запуск аналізу журналу)
logs/                      журнали симуляції та вузлів
logs/lab_7/                журнали QGroundControl (.tlog, .csv) і результати аналізу
```

## Команди

| Команда                 | Призначення                                                                                     |
| ----------------------- | ----------------------------------------------------------------------------------------------- |
| `make help`             | довідка                                                                                         |
| `make image`            | зібрати Docker-образ (перший раз ~15–20 хв)                                                     |
| `make start [ARGS=...]` | запустити Gazebo, SITL, micro-ROS agent і дочекатися готовності до arm                          |
| `make stop`             | зупинити все й видалити контейнер                                                               |
| `QGC=1`                 | до `start`: ще й запустити QGroundControl; до `stop`: закрити його й зберегти `.tlog`           |
| `make shell`            | термінал у контейнері з підключеним середовищем ROS 2                                           |
| `make run <шлях>`       | запустити місію, напр. `make run lab_2/go_to`; шлях відносно `fpv_labs/src`, журнал — у `logs/` |
| `make logs`             | стежити за останнім журналом симуляції                                                          |

Функції з `scripts/lib.sh` можна викликати й напряму (`scripts/lib.sh start rviz:=false`)
або підключити у власний скрипт через `source scripts/lib.sh`.

## Запуск

```bash
make image                    # один раз
make start ARGS=rviz:=false   # без rviz:=false відкриється ще й RViz
make run lab_2/plain_start    # зліт на 2 м, зависання 30 с, посадка
make start ARGS=rviz:=false   # перед кожним польотом — чистий старт
make run lab_2/go_to          # політ до точки варіанта й посадка
make stop
```

Лабораторна 7 (QGroundControl-x86_64.AppImage з docs.qgroundcontrol.com лежить у корені репозиторію):

```bash
make start ARGS=rviz:=false QGC=1   # симуляція + QGroundControl
make run lab_7/flight               # місія AUTO: зліт 50 м -> ціль -> посадка
make stop QGC=1                     # QGC зберігає .tlog у logs/lab_7/Telemetry
make run lab_7/analysis             # таблиця 7.4 і графіки в logs/lab_7/analysis
```

Перевірка вручну в `make shell`:

```bash
ros2 node list
ros2 topic list            # /ap/v1/status, /odometry, ...
ros2 service list | grep /ap/
ros2 topic echo --once /odometry --field pose.pose.position
```
