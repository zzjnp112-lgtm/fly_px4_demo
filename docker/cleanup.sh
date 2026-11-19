#!/bin/bash
# 停止所有仿真/ROS 进程（PX4、gz-sim、桥接、MAVROS、视觉、控制等）
# 用法（在容器内）：
#   docker exec px4sim bash /root/fly_px4_demo/docker/cleanup.sh
# 注意：grep -v cleanup.sh 排除脚本自身（否则 "fly_px4" 会匹配到仓库路径 fly_px4_demo 导致自杀）

echo "停止所有仿真进程 ..."

# 收集所有相关进程 PID
#   grep -v grep      : 排除 grep 命令本身
#   grep -v defunct   : 排除僵尸进程
#   grep -v cleanup.sh: 排除本脚本自身（脚本路径含 fly_px4_demo，会被 fly_px4 误匹配）
pids=$(ps aux | grep -E "gz sim|gzserver|gzclient|bin/px4|make px4|px4_sitl|camer_bridge|time_bridge|point_bridge|mavros_node|fly_px4|qr_node|qr_tracking|ros2 launch|ros2 run|start_tracking_sim|start_gz_sim" \
      | grep -v grep | grep -v defunct | grep -v "cleanup.sh" | awk -v self=$$ '$2 != self {print $2}')

if [ -n "$pids" ]; then
  echo "  杀掉进程: $pids"
  kill -9 $pids 2>/dev/null
else
  echo "  没有发现运行中的仿真进程"
fi

sleep 1

# 统计剩余相关进程（非 defunct）
remaining=$(ps aux | grep -E "gz sim|bin/px4|mavros_node|fly_px4|time_bridge|camer_bridge" \
          | grep -v grep | grep -v defunct | grep -v "cleanup.sh" | wc -l)

echo "清理完成，剩余相关进程: $remaining 个"
