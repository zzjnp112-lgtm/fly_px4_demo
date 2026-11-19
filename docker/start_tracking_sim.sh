#!/bin/bash
# 地面目标追踪仿真：小车(顶面二维码) + 无人机(x500_mid360) + camer_bridge + MAVROS
# 默认带窗口(GUI)；传 headless 或 GUI=0 则无窗口
# 用法：
#   docker exec -it px4sim bash /root/fly_px4_demo/docker/start_tracking_sim.sh          # 默认 GUI
#   docker exec -it px4sim bash /root/fly_px4_demo/docker/start_tracking_sim.sh headless # 无窗口
set -e

REPO=/root/fly_px4_demo
PX4=/root/PX4-Autopilot
WORLD=$REPO/src/simulation_uav/simulation_fly/config/base_world.sdf
WORLD_NAME=base_world
BRIDGE_YAML=$REPO/src/simulation_uav/simulation_fly/config/camera_bridge.yaml

# 1. 模型资源路径（x500_mid360 / car_mixed 都在这）
export GZ_SIM_RESOURCE_PATH=$REPO/src/models:$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds

# 2. ROS2 环境
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash

# 3. gz-sim（默认带窗口，headless 则无窗口）
if [ "${GUI:-1}" = "0" ] || [ "$1" = "headless" ]; then
  echo "[1/5] 启动 gz-sim（headless）..."
  gz sim -s -r "$WORLD" &
else
  echo "[1/5] 启动 gz-sim（带窗口）..."
  gz sim -r "$WORLD" &
fi
sleep 6

# 4. PX4 SITL（无人机控制：make px4_sitl）
#    PX4_GZ_STANDALONE=1：PX4 不自己起 gz-sim，gz_bridge 每 2s 重试 spawn 无人机并监听 14580
#    PX4_GZ_WORLD 必须和 gz-sim 跑的世界名一致（base_world），否则 gz_bridge 连不上
echo "[2/5] 启动 PX4 SITL（gz_x500_mid360）..."
cd "$PX4"
PX4_GZ_WORLD=$WORLD_NAME PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mid360 > /tmp/px4.log 2>&1 &

# 等待 PX4 上线（MAVLink 14580；容器里 ss 对 UDP 显示不准，改用日志确认）
echo "[3/5] 等待 PX4 就绪（MAVLink 14580）..."
for i in $(seq 1 60); do
  if grep -aq 'udp port 14580' /tmp/px4.log 2>/dev/null; then
    echo "      ✓ PX4 已上线"
    break
  fi
  sleep 2
done

# 5. 桥接节点（无人机相机/雷达 → ROS2，喂给检测/FAST-LIO）
echo "[4/5] 启动 time_bridge..."
ros2 run simulation_fly time_bridge &
echo "[4/5] 启动 camer_bridge..."
ros2 run simulation_fly camer_bridge --ros-args --params-file "$BRIDGE_YAML" &

# 6. MAVROS（连接 PX4 SITL，fly_px4 控制用）
echo "[5/5] 启动 MAVROS..."
ros2 launch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14580 \
  namespace:=/mavros &

echo ""
echo "追踪仿真已启动："
echo "  无人机下视相机(看二维码): ros2 topic hz /drone_0_/camera/image_raw"
echo "  无人机前视相机:           ros2 topic hz /drone_0_/front_cam/image_raw"
echo "  无人机雷达点云(FAST-LIO): ros2 topic hz /drone_0_/lidar/points"
echo "  MAVROS 状态:              ros2 topic hz /mavros/local_position/pose"
echo ""
echo "二维码检测节点（另开终端）："
echo "  ros2 run qr_tracking yuan"
echo "  # 前视相机: ros2 run qr_tracking yuan --ros-args -p image_topic:=/drone_0_/front_cam/image_raw"
echo ""
echo "让小车动起来："
echo "  gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.3}'"
echo ""
echo "Ctrl+C 退出。"
wait
