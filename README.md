# Лабораторна робота №2: Gazebo + ArduPilot SITL + ROS 2

Середовище розгорнуте в Docker, бо ROS 2 Jazzy потребує Ubuntu 24.04.

| Компонент | Версія |
|---|---|
| Образ | Ubuntu 24.04.4 LTS (`osrf/ros:jazzy-desktop`), тег `fpv-lab2:jazzy-harmonic` |
| ROS 2 | Jazzy |
| Gazebo Sim | Harmonic (gz-sim 8.15.0, пакунки `ros-jazzy-gz-*-vendor`) |
| ArduPilot SITL | гілка `master`, зібраний з DDS у `/opt/ardu_ws` |
| Python | 3.12.3 |
| Mesa | 25.2.8 (штатна з Ubuntu 24.04) |

## Структура

```
Makefile                   точка входу: `make help`
scripts/lib.sh             бібліотека bash-функцій, які викликає make
docker/Dockerfile          образ (GPU через /dev/dri)
docker/ros2_gz_jazzy.repos репозиторії робочого простору ArduPilot для Jazzy
docker-compose.yml         опис контейнера
fpv_labs/src/              код польотів (звичайні модулі Python, без ROS-пакета і colcon)
  drone.py                   єдиний вузол: зв'язок з ArduPilot і Gazebo, виконання місії по кроках
  handlers.py                бібліотека обробників кроків (arm, зліт, політ до точки, посадка...)
  missions/                  фази місій (takeoff, wait, fly_to, landing) і chain, що з'єднує їх
  lab_2/                     місії лабораторної 2: ланцюжок фаз із параметрами варіанта 21
logs/                      журнали симуляції та вузлів
```

## Команди

| Команда | Призначення |
|---|---|
| `make help` | довідка |
| `make image` | зібрати Docker-образ (перший раз ~15–20 хв) |
| `make start [ARGS=...]` | запустити Gazebo, SITL, micro-ROS agent і дочекатися готовності до arm |
| `make stop` | зупинити все й видалити контейнер |
| `make shell` | термінал у контейнері з підключеним середовищем ROS 2 |
| `make run <шлях>` | запустити місію, напр. `make run lab_2/go_to`; шлях відносно `fpv_labs/src`, журнал — у `logs/` |
| `make logs` | стежити за останнім журналом симуляції |

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

Перевірка вручну в `make shell`:

```bash
ros2 node list
ros2 topic list            # /ap/v1/status, /odometry, ...
ros2 service list | grep /ap/
ros2 topic echo --once /odometry --field pose.pose.position
```

## Відомі особливості

- OpenGL у контейнері працює на штатній Mesa 25.2 з Ubuntu 24.04 і драйвері `iris`:
  Intel Lunar Lake підтримується, вікно Gazebo під XWayland не падає. Обхідний шлях
  з образу на Humble (Mesa з PPA `kisak/turtle` і Zink) більше не потрібен.
- `ros_gz`, `sdformat_urdf` і сам Gazebo Harmonic ставляться з apt-репозиторію ROS
  (у Jazzy це штатна пара), з вихідного коду збираються лише ArduPilot,
  `ardupilot_gazebo`, `ardupilot_gz`, моделі SITL і micro-ROS agent. Список
  репозиторіїв — у `docker/ros2_gz_jazzy.repos`, бо `ros2_gz.repos` з `ardupilot_gz`
  розрахований на Humble.
- Python-пакети в образі системні (PEP 668): `pymavlink` і `dronecan` поставлені через
  `pip --break-system-packages`, MAVProxy — без залежностей, щоб не підмінити системний
  numpy, з яким зібрано пакунки ROS.
- Одразу після старту ArduPilot відхиляє arm («Need Alt Estimate»), поки EKF не
  перейде на GPS; `make start` чекає цього сам. Місія при відмові в arm
  завершується з помилкою, а не повторює спробу.
- `make start` завжди починає з чистого контейнера, тому «залишкових» процесів
  Gazebo/SITL після попереднього запуску не буває.
- Службові файли SITL (eeprom, tlog, dataflash) лежать у `.sitl/`.
