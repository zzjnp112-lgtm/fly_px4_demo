# PX4 SITL + ROS2 + MAVROS 仿真环境 — 命令手册

本目录提供 Docker 仿真环境，在 Ubuntu 20.04 宿主上跑 Ubuntu 22.04 + ROS2 Humble + PX4 SITL + Gazebo Classic + MAVROS。

---

## 一、一次性：装 Docker（宿主，需 sudo）

```bash
# 官方脚本在 20.04 上会因 docker-model-plugin 失败，用下面命令装
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin docker-buildx-plugin

# 前提：先配好 Docker 源（官方脚本已配过就跳过）
# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.asc
# echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu focal stable" | sudo tee /etc/apt/sources.list.d/docker.list

sudo usermod -aG docker $USER
# 重新开终端（让 docker 组生效）
```

验证：
```bash
docker version
```

## 二、构建镜像

```bash
cd /home/zzj/fly_px4_demo
docker build -t px4sim:humble docker/
```

> 首次构建约 20~40 分钟（拉基础镜像 + PX4 子模块），之后有缓存会快很多。

## 三、启动容器

```bash
xhost +local:docker

docker run -d --name px4sim \
  --net=host \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/zzj/fly_px4_demo:/root/fly_px4_demo \
  px4sim:humble sleep infinity
```

> 有 NVIDIA 卡想加速 Gazebo：宿主装 `nvidia-container-toolkit`，加 `--gpus all --runtime nvidia`。

## 四、编译你的包（改代码后要重新跑）

```bash
docker exec px4sim bash -lc "\
  mkdir -p /root/ws/src && \
  ln -sf /root/fly_px4_demo /root/ws/src/px4_control && \
  cd /root/ws && \
  source /opt/ros/humble/setup.bash && \
  colcon build"
```

> `serial` 库已经编进镜像 `/usr/local`，`find_package(serial)` 直接可用，无需再装。

## 五、启动仿真（三个终端，各自 `docker exec -it px4sim bash` 进去）

**终端 A — PX4 + Gazebo（首次会先编译固件和 gazebo 插件，约 15~20 分钟）：**

```bash
docker exec -it px4sim bash -lc \
  "cd /root/PX4-Autopilot && HEADLESS=1 PX4_SIM_MODEL=iris make px4_sitl gazebo-classic"
```

> 想看 Gazebo GUI：去掉 `HEADLESS=1`（X11 已转发回宿主桌面）。

**终端 B — MAVROS：**

```bash
docker exec -it px4sim bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch mavros px4.launch \
     fcu_url:=udp://:14540@127.0.0.1:14580 \
     namespace:=/mavros"
```

> 看到 `CON: Got HEARTBEAT, connected` 即连上。

**终端 C — 主控节点：**

```bash
docker exec -it px4sim bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /root/ws/install/setup.bash && \
   ros2 run px4_control fly_px4"
```

## 六、让无人机起飞（fly_px4 不自发 setpoint/不自己设 OFFBOARD）

fly_px4 的启动逻辑需要「外部先设 OFFBOARD」，且 OFFBOARD 需要持续收到 setpoint 才不掉回。用下面两步：

**① 先喂 setpoint（保持 OFFBOARD 存活）：**

```bash
docker exec px4sim bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic pub -r 30 /mavros/setpoint_position/local \
     geometry_msgs/msg/PoseStamped \
     '{pose: {position: {x: 0.0, y: 0.0, z: 1.1}}}'"
```

**② 设 OFFBOARD 模式（fly_px4 会自己 arm 起飞）：**

```bash
docker exec px4sim bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
     '{custom_mode: OFFBOARD}'"
```

## 七、验证状态

```bash
# 模式/武装
docker exec px4sim bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# 位置（z 到 1.1m 左右说明已起飞）
docker exec px4sim bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/local_position/pose --once"
```

## 八、清理

```bash
docker stop px4sim && docker rm px4sim
```

---

## 关键配置备忘（踩坑记录）

| 项 | 值 |
|---|---|
| Gazebo 包名 | `libgazebo-dev`（**不是** `libgazebo11-dev`） |
| PX4 版本 | v1.15.2（浅克隆 + 补拉主仓/NuttX 标签） |
| MAVROS offboard 端口 | **14580**（v1.15 改了，旧版是 14557） |
| MAVROS 命名空间 | `namespace:=/mavros`（不加则 topic 无前缀，跟 fly_px4 对不上） |
| serial 库 | `github.com/ZhaoXiangBox/serial` 编译到 `/usr/local` |
| GeographicLib | `install_geographiclib_datasets.sh` 装了 geoids/gravity/magnetic |
| PX4 Python 依赖 | 含 `pyros-genmsg`（提供 genmsg） |
| fly_px4 的 Arduino 串口 | 已改成仿真安全（`serinit` 打开失败静默跳过） |
