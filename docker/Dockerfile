# PX4 SITL + ROS2 Humble + MAVROS 仿真环境
# 宿主: Ubuntu 20.04 (Docker 内为 Ubuntu 22.04 / jammy)
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 1) Gazebo Classic 11 + GStreamer（PX4 SITL gazebo-classic 依赖）
#    注意：jammy 的包名是 libgazebo-dev（不是 libgazebo11-dev）
RUN apt-get update && apt-get install -y \
    gazebo libgazebo-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# 2) PX4 编译依赖（pyros-genmsg 提供 genmsg 模块，必须装）
RUN apt-get update && apt-get install -y \
    git wget curl python3-pip python3-jinja2 python3-yaml \
    python3-numpy python3-toml ninja-build exiftool xxd \
    && pip3 install --user empy packaging kconfiglib jsonschema future pyros-genmsg \
    && rm -rf /var/lib/apt/lists/*

# 3) MAVROS（ROS2）+ GeographicLib 地理模型数据集（mavros 必需）
RUN apt-get update && apt-get install -y \
    ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
    geographiclib-tools \
    && rm -rf /var/lib/apt/lists/* \
    && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# 4) serial 库（wjwwood/serial API；ROS2 无 apt 包，用镜像源编译到 /usr/local）
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /tmp && git clone --depth 1 https://github.com/ZhaoXiangBox/serial.git serial && \
    cd serial && mkdir -p build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j4 && make install && \
    rm -rf /tmp/serial"

# 5) PX4-Autopilot 源码（浅克隆 + 补拉标签，否则版本脚本 build_git_version.h 报错）
WORKDIR /root
RUN git clone --depth 1 --branch v1.15.2 \
        https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git submodule update --init --recursive --depth 1 && \
    git fetch --tags --depth 1 origin && \
    cd platforms/nuttx/NuttX/nuttx && git fetch --tags --depth 1 origin

# 6) 工作区（用户工程运行时挂载）
RUN mkdir -p /root/ws/src
WORKDIR /root
