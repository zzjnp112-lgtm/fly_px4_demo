#!/bin/bash
# 地面目标追踪仿真：小车(顶面二维码) + 无人机(x500_mid360) + camer_bridge 前置链路
# 用法：
#   docker exec -it px4sim bash /root/fly_px4_demo/docker/start_tracking_sim.sh
#   GUI=1 docker exec -it px4sim bash /root/fly_px4_demo/docker/start_tracking_sim.sh   # 带窗口
set -e

REPO=/root/fly_px4_demo
WORLD=$REPO/src/simulation_uav/simulation_fly/config/simple_world.sdf
BRIDGE_YAML=$REPO/src/simulation_uav/simulation_fly/config/camera_bridge.yaml

# 1. 模型资源路径（x500_mid360 / car_mixed 都在这）
export GZ_SIM_RESOURCE_PATH=$REPO/src/models:/root/PX4-Autopilot/Tools/simulation/gz/models

# 2. ROS2 环境
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash

# 3. gz-sim 加载简单世界（世界内已 include 无人机 + 小车，无需 spawn）
#    支持 GUI=1 环境变量 或 GUI=1 位置参数
if [ "${GUI:-0}" = "1" ] || [ "$1" = "GUI=1" ] || [ "$1" = "gui" ]; then
  echo "[1/3] 启动 gz-sim（带窗口）..."
  gz sim -r "$WORLD" &
else
  echo "[1/3] 启动 gz-sim（headless）..."
  gz sim -s -r "$WORLD" &
fi
sleep 6

# 4. 桥接节点（无人机相机/雷达 → ROS2，喂给检测/FAST-LIO）
echo "[2/3] 启动 time_bridge..."
ros2 run simulation_fly time_bridge &
echo "[3/3] 启动 camer_bridge..."
ros2 run simulation_fly camer_bridge --ros-args --params-file "$BRIDGE_YAML" &

echo ""
echo "追踪仿真已启动。前置链路（相机/雷达 → ROS2）就绪："
echo "  无人机下视相机(看小车二维码): ros2 topic hz /drone_0_/camera/image_raw"
echo "  无人机前视相机:               ros2 topic hz /drone_0_/front_cam/image_raw"
echo "  无人机雷达点云(FAST-LIO):     ros2 topic hz /drone_0_/lidar/points"
echo "  无人机雷达 IMU:               ros2 topic hz lidar/imu/data"
echo ""
echo "让小车动起来："
echo "  gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.3}'"
echo ""
echo "Ctrl+C 退出。"
wait
