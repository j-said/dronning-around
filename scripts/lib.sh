#!/usr/bin/env bash

set -euo pipefail

LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER=fpv_lab2

# QGroundControl на хості: QGC=1 для start і stop
QGC_APP="$LAB_DIR/QGroundControl-x86_64.AppImage"
QGC_PROCESS="QGroundControl"  # назва процесу всередині AppImage
QGC_LOG_DIR="${QGC_LOG_DIR:-$LAB_DIR/logs/lab_7}"

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export RENDER_GID="$(getent group render | cut -d: -f3)"
export VIDEO_GID="$(getent group video | cut -d: -f3)"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

compose() {
    docker compose -f "$LAB_DIR/docker-compose.yml" "$@"
}

# in_container [прапорці docker exec] <команда>
in_container() {
    local flags=()
    while [[ "${1:-}" == -* ]]; do
        flags+=("$1")
        shift
    done
    docker exec "${flags[@]}" "$CONTAINER" bash -c "source ~/.bashrc_lab && $*"
}

ensure_up() {
    docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || compose up -d
}

wait_for() {
    local message="$1" command="$2"
    echo -n "$message"
    for _ in $(seq 1 60); do
        if in_container "$command" >/dev/null 2>&1; then
            echo " — готово"
            return 0
        fi
        echo -n "."
        sleep 2
    done
    echo
    echo "Не дочекалися, перевірте журнал ${LOG_FILE:-}" >&2
    return 1
}

image() {
    compose build "$@"
}

start() {
    local launch_file="${LAUNCH_FILE:-iris_maze.launch.py}"
    LOG_FILE="logs/sim_$(date +%Y%m%d_%H%M%S).log"

    compose down --remove-orphans >/dev/null 2>&1 || true
    compose up -d
    mkdir -p "$LAB_DIR/logs"
    in_container -d "mkdir -p /workspace/.sitl && cd /workspace/.sitl \
        && ros2 launch ardupilot_gz_bringup $launch_file $* > /workspace/$LOG_FILE 2>&1"
    echo "Симуляцію запущено, журнал: $LOG_FILE"
    # QGC сам підключиться, щойно SITL почне слати HEARTBEAT
    if [[ "${QGC:-0}" == 1 ]]; then
        start_qgc
    fi

    wait_for "Очікування ArduPilot (тема /ap/v1/status)" \
        "timeout 5 ros2 topic echo --once /ap/v1/status"
    # arm відхиляється ("Need Alt Estimate"), поки EKF не перейде на GPS
    wait_for "Очікування готовності EKF до arm" \
        "grep -q 'EKF3 IMU1 is using GPS' /workspace/$LOG_FILE && timeout 5 ros2 service call /ap/v1/prearm_check std_srvs/srv/Trigger | grep -q 'success=True'"
    sleep 10
    echo "Далі: make run lab_2/plain_start | make run lab_2/go_to | make shell | make stop"
}

stop() {
    compose down --remove-orphans
    if [[ "${QGC:-0}" == 1 ]]; then
        stop_qgc
    fi
}

start_qgc() {
    if pgrep -x "$QGC_PROCESS" >/dev/null; then
        echo "QGroundControl уже запущено"
        return 0
    fi
    if [[ ! -x "$QGC_APP" ]]; then
        echo "Немає $QGC_APP; завантажте AppImage з docs.qgroundcontrol.com" >&2
        return 1
    fi
    mkdir -p "$LAB_DIR/logs"
    setsid "$QGC_APP" >"$LAB_DIR/logs/qgc_$(date +%Y%m%d_%H%M%S).log" 2>&1 </dev/null &
    echo "QGroundControl запущено"
}

stop_qgc() {
    pgrep -x "$QGC_PROCESS" >/dev/null || return 0
    pkill -TERM -x "$QGC_PROCESS"
    echo -n "Закриття QGroundControl"
    for _ in $(seq 1 15); do
        if ! pgrep -x "$QGC_PROCESS" >/dev/null; then
            echo " — готово"
            save_qgc_tlog
            return 0
        fi
        echo -n "."
        sleep 1
    done
    # SIGTERM лише відкриває діалог підтвердження закриття: завершуємо примусово,
    # тимчасовий журнал при цьому лишається цілим
    pkill -KILL -x "$QGC_PROCESS" || true
    sleep 1
    echo " — завершено примусово"
    save_qgc_tlog
}

# QGC зберігає .tlog, коли від'єднується апарат; UDP-зв'язок сам не рветься,
# тож без підтвердженого закриття журнал лишається в /tmp: переносимо його самі
save_qgc_tlog() {
    local temp target
    mkdir -p "$QGC_LOG_DIR/Telemetry"
    for temp in /tmp/FlightData*.mavlink; do
        [[ -s "$temp" ]] || continue
        target="$QGC_LOG_DIR/Telemetry/$(date -r "$temp" '+%Y-%m-%d %H-%M-%S').tlog"
        mv "$temp" "$target"
        echo "Журнал: ${target#"$LAB_DIR"/}"
    done
}

shell() {
    ensure_up
    docker exec -it "$CONTAINER" bash
}

# run <шлях>: lab_2/go_to, lab_2/go_to.py або fpv_labs/src/lab_2/go_to.py
run() {
    local part="${1:?Вкажіть місію, наприклад: make run lab_2/go_to}"
    part="${part#fpv_labs/src/}"
    part="${part%.py}"
    if [[ ! -f "$LAB_DIR/fpv_labs/src/$part.py" ]]; then
        echo "Немає файлу fpv_labs/src/$part.py" >&2
        return 1
    fi
    local module="${part//\//.}"
    local log_file="logs/${module}_$(date +%Y%m%d_%H%M%S).log"
    ensure_up
    mkdir -p "$LAB_DIR/logs"
    in_container -t "cd /workspace/fpv_labs/src && python3 -m $module" | tee "$LAB_DIR/$log_file"
    echo "Журнал: $log_file"
}

logs() {
    tail -f "$(ls -t "$LAB_DIR"/logs/sim_*.log | head -1)"
}

# Копія Python-пакетів ROS 2 з образу в .pydeps/ — лише для підказок і перевірок у редакторі
pydeps() {
    local id dir
    id="$(docker create fpv-lab2:jazzy-harmonic)"
    rm -rf "$LAB_DIR/.pydeps"
    mkdir -p "$LAB_DIR/.pydeps"
    for dir in ros_site:/opt/ros/jazzy/lib/python3.12/site-packages \
        ardupilot:/opt/ardu_ws/install/ardupilot_msgs/lib/python3.12/site-packages; do
        docker cp -q "$id:${dir#*:}" "$LAB_DIR/.pydeps/${dir%%:*}"
    done
    # Окремі пакети з dist-packages: каталоги цілком займають ~750 МБ
    mkdir -p "$LAB_DIR/.pydeps/python"
    for dir in /usr/local/lib/python3.12/dist-packages/pymavlink \
        /usr/lib/python3/dist-packages/numpy \
        /usr/lib/python3/dist-packages/matplotlib \
        /usr/lib/python3/dist-packages/pandas; do
        docker cp -q "$id:$dir" "$LAB_DIR/.pydeps/python/" 2>/dev/null \
            || echo "Немає $dir в образі; перезберіть його: make image" >&2
    done
    docker rm "$id" >/dev/null
    echo "Готово: .pydeps/ ($(du -sh "$LAB_DIR/.pydeps" | cut -f1))"
}

[[ "${BASH_SOURCE[0]}" != "$0" ]] || "$@"
