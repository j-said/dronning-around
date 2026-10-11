# Лабораторна робота №2: Gazebo + ArduPilot SITL + ROS 2

Середовище розгорнуте в Docker, бо ROS 2 Humble потребує Ubuntu 22.04.

| Компонент | Версія |
|---|---|
| Образ | Ubuntu 22.04.5 LTS (`osrf/ros:humble-desktop`) |
| ROS 2 | Humble |
| Gazebo Sim | Harmonic (gz-sim 8.15.0) |
| ArduPilot SITL | гілка `master`, зібраний з DDS у `/opt/ardu_ws` |
| Python | 3.10.12 |

## Структура

```
Makefile                   точка входу: `make help`
scripts/lib.sh             бібліотека bash-функцій, які викликає make
docker/Dockerfile          образ (GPU через /dev/dri)
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
| `make image` | зібрати Docker-образ (перший раз ~30–40 хв) |
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

- OpenGL у контейнері працює через Zink (Vulkan) і Mesa 25 з PPA `kisak/turtle`.
  Зі штатною Mesa 23.2 немає драйвера для Intel Lunar Lake, а з драйвером `iris`
  вікно Gazebo падає під XWayland.
- Одразу після старту ArduPilot відхиляє arm («Need Alt Estimate»), поки EKF не
  перейде на GPS; `make start` чекає цього сам. Місія при відмові в arm
  завершується з помилкою, а не повторює спробу.
- `make start` завжди починає з чистого контейнера, тому «залишкових» процесів
  Gazebo/SITL після попереднього запуску не буває.
- Службові файли SITL (eeprom, tlog, dataflash) лежать у `.sitl/`.
