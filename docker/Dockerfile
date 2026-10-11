FROM osrf/ros:jazzy-desktop

ARG USER_NAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    GZ_VERSION=harmonic \
    LANG=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- базові інструменти та залежності збирання ArduPilot ---
# JDK 17: Gradle 7.6 з Micro-XRCE-DDS-Gen не працює з типовим для 24.04 JDK 21
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget gnupg lsb-release software-properties-common sudo git \
    build-essential ccache gawk rsync nano less procps psmisc iproute2 \
    openjdk-17-jdk-headless \
    python3-pip python3-dev python3-vcstool python3-rosdep \
    python3-colcon-common-extensions \
    python3-empy python3-pexpect python3-future python3-lxml \
    python3-numpy python3-setuptools python3-wxgtk4.0 python3-matplotlib \
    python3-serial \
    libxml2-dev libxslt1-dev \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*
ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
ENV PATH="${JAVA_HOME}/bin:${PATH}"

# --- Gazebo Harmonic і міст ROS 2 <-> Gazebo ---
# У Jazzy Harmonic - штатна версія: бібліотеки Gazebo йдуть пакунками ros-jazzy-gz-*-vendor
# з репозиторію ROS. Окремий gz-harmonic з репозиторію OSRF не ставимо, щоб у системі
# не було двох копій тих самих бібліотек.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-sdformat-urdf \
    ros-jazzy-gz-cmake-vendor ros-jazzy-gz-common-vendor ros-jazzy-gz-plugin-vendor \
    ros-jazzy-gz-rendering-vendor ros-jazzy-gz-sim-vendor ros-jazzy-gz-tools-vendor \
    && rm -rf /var/lib/apt/lists/*

# --- Python-залежності ArduPilot / MAVProxy ---
# В apt для 24.04 цих пакунків немає, а pip без прапорця відмовляється ставити в систему (PEP 668).
# MAVProxy ставимо без залежностей: інакше pip тягне numpy 2 і свіжий matplotlib поверх системних,
# з якими зібрано пакунки ROS; потрібні йому numpy, matplotlib, wx, serial, lxml уже є з apt.
RUN pip3 install --no-cache-dir --break-system-packages pymavlink dronecan \
    && pip3 install --no-cache-dir --break-system-packages --no-deps MAVProxy

# --- Micro-XRCE-DDS-Gen (генератор DDS-типів для AP_DDS) ---
RUN git clone --recurse-submodules --depth 1 \
    https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git /opt/Micro-XRCE-DDS-Gen \
    && cd /opt/Micro-XRCE-DDS-Gen && ./gradlew assemble --no-daemon \
    && rm -rf /root/.gradle
ENV PATH="${PATH}:/opt/Micro-XRCE-DDS-Gen/scripts"

# --- користувач з тим самим UID, що й на хості (права на змонтовані файли, X11) ---
# В образах Ubuntu 24.04 уже є користувач ubuntu з UID/GID 1000 - прибираємо, щоб не заважав
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd -g ${USER_GID} ${USER_NAME} \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && mkdir -p /opt/ardu_ws/src /workspace \
    && chown -R ${USER_NAME}:${USER_NAME} /opt/ardu_ws /workspace

USER ${USER_NAME}
WORKDIR /opt/ardu_ws

# --- вихідний код: ArduPilot, ardupilot_gazebo, ardupilot_gz, моделі SITL, micro-ROS agent ---
COPY --chown=${USER_UID}:${USER_GID} ros2_gz_jazzy.repos /opt/ardu_ws/ros2_gz_jazzy.repos
RUN vcs import --shallow --input ros2_gz_jazzy.repos src

RUN for repo in src/*/; do \
    [ -f "$repo/.gitmodules" ] || continue; \
    for attempt in 1 2 3 4 5 6 7 8; do \
    git -C "$repo" submodule update --init --recursive --depth 1 --jobs 1 && break; \
    [ "$attempt" = 8 ] && exit 1; \
    sleep 60; \
    done; \
    done

# Ключі gz-* пропускаємо: Gazebo вже встановлено vendor-пакунками вище
RUN sudo apt-get update && rosdep update \
    && source /opt/ros/jazzy/setup.bash \
    && rosdep install --from-paths src --ignore-src -y -r \
    --skip-keys "gz-cmake3 gz-common5 gz-plugin2 gz-sim8" \
    && sudo rm -rf /var/lib/apt/lists/*

# --- збирання робочого простору ArduPilot ---
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS="-j6" colcon build --parallel-workers 2 \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && rm -rf log

COPY --chown=${USER_UID}:${USER_GID} bashrc_lab /home/${USER_NAME}/.bashrc_lab
RUN echo "source ~/.bashrc_lab" >> /home/${USER_NAME}/.bashrc

WORKDIR /workspace
CMD ["sleep", "infinity"]
