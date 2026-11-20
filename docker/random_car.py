#!/usr/bin/env python3
"""
让小车随机乱走（随机线速度 + 随机角速度），用于对比 PID 和 MPC 追踪效果。
用法（容器内）：
  python3 /root/fly_px4_demo/docker/random_car.py            # 默认 60 秒
  python3 /root/fly_px4_demo/docker/random_car.py 30         # 走 30 秒
Ctrl+C 随时停止（停止时自动把小车速度清零）。
"""
import random
import subprocess
import sys
import time

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

# 随机范围
LIN_MIN, LIN_MAX = -1.2, 1.2     # 线速度 m/s（小车限幅 ±0.5）
ANG_MIN, ANG_MAX = -2.0, 2.0     # 角速度 rad/s（转向）
DUR_MIN, DUR_MAX = 2.0, 2.5      # 每个速度指令持续时长 s


def cmd(vx, wz):
    subprocess.run(
        ["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
         "-p", f"linear: {{x: {vx}}} angular: {{z: {wz}}}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    print(f"[random_car] 小车随机乱走 {DURATION:.0f} 秒，Ctrl+C 停止")
    end = time.time() + DURATION
    try:
        while time.time() < end:
            vx = round(random.uniform(LIN_MIN, LIN_MAX), 2)
            wz = round(random.uniform(ANG_MIN, ANG_MAX), 2)
            dur = round(random.uniform(DUR_MIN, DUR_MAX), 2)
            print(f"  vx={vx:+.2f} m/s   wz={wz:+.2f} rad/s   (持续 {dur:.2f}s)")
            cmd(vx, wz)
            time.sleep(dur)
    finally:
        cmd(0.0, 0.0)
        print("[random_car] 结束，小车已停")


if __name__ == "__main__":
    main()
