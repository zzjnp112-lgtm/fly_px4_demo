#!/bin/bash
# gz-sim + x500_mid360 + camer_bridge 一键启动（在容器内运行）
# 用法：
#   docker exec -it px4sim bash /root/fly_px4_demo/docker/start_gz_sim.sh          # headless
#   GUI=1 docker exec -it px4sim bash /root/fly_px4_demo/docker/start_gz_sim.sh   # 带窗口
set -e

REPO=/root/fly_px4_demo
WORLD=$REPO/src/simulation_uav/simulation_fly/config/custom_world.sdf
MODEL=$REPO/src/models/x500_mid360/model.sdf
BRIDGE_YAML=$REPO/src/simulation_uav/simulation_fly/config/camera_bridge.yaml

# 1. 模型资源路径（x500_mid360 / x500 / x500_base / gemini2 都在这里）
export GZ_SIM_RESOURCE_PATH=$REPO/src/models:/root/PX4-Autopilot/Tools/simulation/gz/models

# 2. ROS2 环境
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash

# 3. 启动 gz-sim
if [ "${GUI:-0}" = "1" ]; then
  echo "[1/4] 启动 gz-sim（带窗口）..."
  gz sim -r "$WORLD" &
else
  echo "[1/4] 启动 gz-sim（headless）..."
  gz sim -s -r "$WORLD" &
fi

# 4. 等世界加载完再 spawn
echo "[2/4] 等待 gz-sim 加载..."
sleep 8

echo "[3/4] spawn x500_mid360 无人机模型..."
gz service -s /world/custom_world/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req "sdf_filename: \"$MODEL\", name: \"x500_mid360\""

# 5. 启动桥接节点
echo "[4/4] 启动 time_bridge + camer_bridge..."
ros2 run simulation_fly time_bridge &
ros2 run simulation_fly camer_bridge --ros-args --params-file "$BRIDGE_YAML" &

echo ""
echo "全部启动完成。验证数据流："
echo "  ros2 topic hz /drone_0_/lidar/points       # 雷达点云（FAST-LIO 用）"
echo "  ros2 topic hz /drone_0_/camera/image_raw   # 下视相机"
echo "  ros2 topic hz lidar/imu/data               # 雷达 IMU"
echo "  ros2 topic hz /drone_0_/front_cam/image_raw # 前视相机"
echo ""
echo "Ctrl+C 退出。"

wait
