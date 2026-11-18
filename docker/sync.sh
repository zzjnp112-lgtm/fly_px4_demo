#!/bin/bash
# 同步宿主机代码改动到 docker 环境
# 功能：1) 启动容器(若停止)  2) 重新编译 ROS2 包  3) 小车模型有改动则重新转换 URDF→SDF
# 用法：bash docker/sync.sh
set -e

CONTAINER=px4sim

# ---------- 1. 启动容器 ----------
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "[1/3] 容器已在运行"
else
  echo "[1/3] 启动容器 $CONTAINER ..."
  docker start "$CONTAINER" >/dev/null
  sleep 2
fi

# ---------- 2. 重新编译 ROS2 包 ----------
echo "[2/3] 重新编译 ROS2 包 ..."
docker exec "$CONTAINER" bash -lc "
  cd /root/ws && source /opt/ros/humble/setup.bash
  colcon build
" 2>&1 | tail -15

# ---------- 3. 小车模型同步 ----------
echo "[3/3] 检查小车模型是否需重新转换 ..."
docker exec "$CONTAINER" bash -lc "
  cd /root/fly_px4_demo/src/models
  if [ car_modles/car_mixed.urdf -nt car_mixed/model.sdf ]; then
    echo '  URDF 有更新，重新转换 SDF ...'
    gz sdf -p car_modles/car_mixed.urdf > car_mixed/model.sdf
    # URDF→SDF 转换会丢二维码纹理，用 python 补回来
    python3 - << 'PYEOF'
p = 'car_mixed/model.sdf'
s = open(p).read()
old = '''<material>
          <diffuse>0 0 0 1</diffuse>
          <ambient>0 0 0 1</ambient>
        </material>'''
new = '''<material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 0</specular>
          <pbr>
            <metal>
              <albedo_map>model://car_mixed/materials/textures/qr_code.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>'''
if old in s:
    s = s.replace(old, new, 1)
    open(p, 'w').write(s)
    print('  二维码纹理已补回')
else:
    print('  警告：未匹配到二维码材质，请手动检查 car_mixed/model.sdf')
PYEOF
  else
    echo '  小车 SDF 已是最新，无需转换'
  fi
"

echo ""
echo "同步完成 ✅"
echo "重启仿真：docker exec -it $CONTAINER bash /root/fly_px4_demo/docker/start_tracking_sim.sh GUI=1"
