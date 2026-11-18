# 地面动态目标追踪 —— 单目 AprilTag + 卡尔曼滤波 + MPC 方案

> 版本：v1（草案）
> 日期：2026-11-13
> 目标平台：PX4（MAVROS / OFFBOARD）+ ROS2（rclcpp）
> 追踪对象：贴有 AprilTag/二维码的地面小车

---

## 1. 现状梳理

当前系统由两个 ROS2 节点 + 三个控制库组成，本质是「像素误差 PID 的 IBVS（基于图像的视觉伺服）」。

### 1.1 定位 / 视觉位姿节点 `src/fuck_mid360.cpp`

- 订阅 `/Odometry`（Mid-360 雷达 SLAM 里程计，`nav_msgs/Odometry`）
- 发布 `/mavros/vision_pose/pose`（喂给 PX4 EKF2 做视觉融合）
- 发布调试量 `/pose_mid_x|y|z`、`pose_mid_yaw`

### 1.2 主控节点 `src/fly_px4.cpp`

- `MavrosSubscriber`：订阅 `/mavros/state`、`/mavros/local_position/pose`
- `PidRead` / `FixedPoint`：订阅目标**像素点** `/zzjnp`（`geometry_msgs/Point`），把像素误差送入 XY 两个 PID，输出限幅 ±0.3 的速度指令
- `Task`：20ms 状态机 `TAKEOFF → FLYING → TRACKING → LANDING`
- 追踪段 `handle_tracking` 直接发布**机体系速度** `/mavros/setpoint_velocity/cmd_vel_unstamped`

### 1.3 现有方案的问题

1. 只有像素误差，没有目标 3D 位置、速度、预测，目标一加速就滞后/超调，容易甩出画面。
2. 无航向协同，机身不朝向目标，视觉目标易越出相机视场。
3. 追踪段存在逻辑隐患（浮点判 0、位置/速度指令冲突）。
4. 雷达只用于自身定位，未用于地面目标检测。

---

## 2. 目标与总体方案

以 **MPC（模型预测控制）** 取代现有像素 PID，实现地面动态目标的预测性、带约束、带航向协同的追踪。

核心链路：

```
单目相机 + AprilTag
      │ 检测角点
      ▼
solvePnP（内参 K + 已知码尺寸）→ 码在相机系 6-DoF 位姿
      │ 相机→机体→局部ENU 坐标变换（时间戳对齐）
      ▼
卡尔曼滤波（2D 匀速 CV，局部 ENU 系）
      │ 平滑 + 估速度 + 去野值 + 丢帧桥接 + N 步预测
      ▼
MPC 控制器（参考轨迹 = 目标预测）
      │ 输出速度/加速度指令
      ▼
MAVROS setpoint_raw/local（速度 + yaw）
```

MPC 相比 PID 的增量价值：

- **预测**：知道目标下一步去哪，提前机动，消除滞后。
- **硬约束**：速度/加速度上限、高度下限、地理围栏。
- **视野（FOV）约束**：把目标保持住相机视场锥内，降低丢失概率。
- **航向协同**：yaw 朝向目标。

---

## 3. 系统架构与数据流

```
┌────────────────────────────┐
│ 相机驱动 + AprilTag 检测    │   image_raw / camera_info
│ (apriltag3 / OpenCV aruco) │
└──────────────┬─────────────┘
               │ tag 角点
               ▼
┌────────────────────────────┐
│  PnP 位姿解算               │   已知码尺寸 + 内参
└──────────────┬─────────────┘
               │ 相机系 6-DoF
               ▼
┌────────────────────────────┐
│  坐标变换（TF）             │   相机→机体外参 + /mavros/local_position
│  → 局部 ENU 系             │
└──────────────┬─────────────┘
               │ 目标 3D 量测 (px, py, 可选 z/航向)
               ▼
┌────────────────────────────┐
│  target_tracker 节点        │   Kalman 滤波（CV）
│  → 发布 TargetState         │   position + velocity + cov + lost_flag
└──────────────┬─────────────┘
               │ 目标状态 + N 步预测
               ▼
┌────────────────────────────┐
│  MPC 控制器节点             │   无人机状态 + 目标预测轨迹
│  → 速度/加速度 + yaw 指令   │
└──────────────┬─────────────┘
               ▼
┌────────────────────────────┐
│  MAVROS setpoint_raw/local  │   PX4 外环
└────────────────────────────┘
```

自定义消息建议：

```
# msg/TargetState.msg
std_msgs/Header header
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64[36] covariance   # 可选
bool lost
```

---

## 4. 目标感知：单目 + AprilTag

### 4.1 为什么单目可行

普通单目的问题是**尺度歧义**（分不清「远的大物体」和「近的小物体」）。二维码/标记物的**物理尺寸已知**，配合标定好的内参，检测到码的 4 个角点后跑一次 **solvePnP** 即可解出码相对相机的 6-DoF 位姿（含 3D 位置），本质是把已知大小的标记当标定板用。

### 4.2 标记选择

- **推荐 AprilTag（tag36h11）或 ArUco**，不要普通二维码。二者专为位姿估计设计，检测更快、更抗角度/距离。
- tag 尺寸按追踪距离估算：720p、HFOV≈70°、f≈914px 下，20cm tag 在 3m≈61px（稳）、8m≈23px（临界）。追踪 5m+ 建议 25–30cm tag，最小检测像素阈值设 15–20px。

### 4.3 检测流程

```
图像 → 检测 tag 角点（校验 ID）→ solvePnP(K, S) → 码在相机系位姿 T_c^t
     → T_b^c（相机→机体外参，固定安装标定一次）
     → T_w^b（来自 /mavros/local_position 姿态+位置）
     → 码在局部 ENU 的 3D 位置（+ 朝向）
```

AprilTag 还额外给**目标航向**（R），可作为转弯检测 / CT 模型的辅助信号。

### 4.4 局限与注意

| 问题 | 对策 |
|---|---|
| 视野/距离受限 | 加大 tag、调最小像素阈值 |
| 码面夹角 >60–70° 失败 | 无人机尽量保持对头，MPC 加 FOV 约束 |
| 遮挡/运动模糊/卷帘快门 | KF 预测桥接 + 提高快门/光照 |
| 检测延迟 5–30ms | 用图像采集时间戳进 KF，别用 wall-clock |
| 远距离 PnP 噪声大 | KF 距离自适应 R，或与 Mid-360 聚类融合 |

---

## 5. 坐标变换（易错点）

- 统一用**局部 ENU 系**，与 `/mavros/local_position` 一致。
- 明确 NED↔ENU、机体↔局部、相机↔机体的转换方向，建议阶段 0 就写单测。
- **时间戳对齐**：图像要带采集时间戳，用同一时刻（或插值）的无人机位姿做变换，否则引入额外误差。

---

## 6. 卡尔曼滤波（target_tracker）

### 6.1 坐标系选型

- **局部 ENU（推荐）**：变换后再滤波，目标物理运动规律（CV/CT）才成立，MPC 参考与无人机状态同系对齐。
- **相机/机体系（不推荐）**：需每帧补偿 ego-motion，复杂且易错。

### 6.2 模型：2D 匀速（CV）起步

地面小车贴地运动，z 基本不变，水平面用 2D CV，z 单独低通/常量处理。

状态 `x = [px, py, vx, vy]ᵀ`（局部 ENU 平面）

**运动模型（DWNA）：**

```
x_{k+1} = F x_k + w,  w ~ N(0, Q)

    [1 0 dt 0]
F = [0 1 0 dt]
    [0 0 1  0]
    [0 0 0  1]
```

**量测模型：**

```
z_k = H x_k + v,  H = [1 0 0 0; 0 1 0 0],  v ~ N(0, R)
```

`z` 为 AprilTag PnP 变换到局部 ENU 后的 (px, py)。

- 起步：CV + 略大过程噪声（容忍小机动）。
- 目标频繁急转：升级 CT（恒转）或 IMM，AprilTag 目标航向做辅助。

### 6.3 量测噪声 R：随距离变化

PnP 误差随距离近似平方增长，固定 R 不合理：

```
R = R0 * (1 + (range / range0)^2) * I
```

或用检测质量做代理（像素面积、角点数、重投影误差、`decision_margin`）映射成 R。

### 6.4 丢帧 / 遮挡

```
有检测 → predict(dt) + update(z)   lost = 0
无检测 → 只 predict(dt)            lost++（P 膨胀，表示不确定性增长）
```

- `lost < N1`（0.2–0.5s）：纯预测输出，MPC 继续按预测追。
- `lost > N2`（≈1s）：判定目标丢失，MPC 退出 → 悬停/返航。

### 6.5 野值 / 错检

- **ID 校验**：只认贴在小车上的 tag ID。
- **马氏距离门**：`d² = yᵀ S⁻¹ y`（`S = HPHᵀ + R`），超过 `χ²(0.99, 2)≈9.2` 判为野值丢弃。

### 6.6 C++ 骨架

```cpp
#include <Eigen/Dense>

struct CVKalman {
  Eigen::Matrix4d F, P, Q;
  Eigen::Matrix<double,2,4> H;
  Eigen::Matrix2d R, I2;
  Eigen::Vector4d x;            // [px,py,vx,vy] 局部ENU
  int    lost = 0;
  double q;                     // 加速度谱密度，调参

  CVKalman(double px, double py, double accel_noise) : q(accel_noise) {
    x << px, py, 0, 0;
    P = Eigen::Matrix4d::Identity() * 1.0;
    H << 1,0,0,0,
         0,1,0,0;
    I2 = Eigen::Matrix2d::Identity();
    buildQ(0.05);
  }

  void buildQ(double dt) {
    double d2=dt*dt, d3=d2*dt, d4=d2*d2;
    Q << d4/4, 0,    d3/2, 0,
         0,    d4/4, 0,    d3/2,
         d3/2, 0,    d2,   0,
         0,    d3/2, 0,    d2;
    Q *= q;
  }

  void predict(double dt) {
    F = Eigen::Matrix4d::Identity();
    F(0,2)=F(1,3)=dt;
    buildQ(dt);
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(const Eigen::Vector2d& z, double range, double R0, double range0) {
    R = R0 * (1.0 + (range*range)/(range0*range0)) * I2;
    Eigen::Vector2d y = z - H*x;
    Eigen::Matrix2d S = H*P*H.transpose() + R;
    double d2 = y.transpose() * S.inverse() * y;
    if (d2 > 9.2) { lost++; return; }     // 野值丢弃
    Eigen::Matrix<double,4,2> K = P * H.transpose() * S.inverse();
    x += K * y;
    P -= K * H * P;
    lost = 0;
  }

  Eigen::Vector2d predictN(int n, double dt) {   // 给 MPC 的 N 步前推
    Eigen::Matrix4d Fn = Eigen::Matrix4d::Identity();
    Fn(0,2)=Fn(1,3)=dt*n;
    return (Fn * x).head<2>();
  }
};
```

### 6.7 调参建议

| 参数 | 作用 | 初值建议 |
|---|---|---|
| `q` | 目标机动容忍度 | 0.5–2.0，滞后就调大 |
| `R0` | 近处量测噪声 | 按 PnP 标定残差，几 cm² 量级 |
| `range0` | R 放大拐点 | 3–5m |
| 马氏门限 | 野值灵敏度 | 9.2（99%），漏检多降 5.99（95%） |
| `lost` 阈值 | 丢目标判定 | N2≈1s |

---

## 7. MPC 控制器

### 7.1 模型

PX4 已提供内环（姿态/角速度）与外环（速度/加速度）接口，MPC 作为**外环引导**，将无人机近似为每轴双积分器或运动学速度模型。

- 状态 `x = [px, py, pz, vx, vy, vz]ᵀ`
- 输入 `u = [ax, ay, az]ᵀ`（yaw 单独给定）
- 离散线性化模型 `x_{k+1} = A x_k + B u_k`

### 7.2 代价函数（滚动时域 N 步）

```
J = Σ (x_k - r_k)ᵀ Q (x_k - r_k)   ← 追踪误差（对预测目标点 r_k）
  + Σ u_kᵀ R u_k                   ← 控制量
  + Σ (v_k - v_target_k)ᵀ W (…)    ← 相对速度（平滑拦截）
```

其中参考轨迹 `r_{k+1..k+N}` = KF 的 `predictN`（目标匀速外推）。

### 7.3 约束

- 速度上限、加速度上限
- 高度下限（不贴地）
- **FOV 锥约束**：相对视线角保持在相机视场内（非凸，需线性化/软约束）
- 地理围栏

### 7.4 求解

- 线性 MPC → QP，用 OSQP（嵌入式 C++）或 qpOASES/ACADO。
- 规模：4–6 状态、2–3 输入、10–20 步，20–50Hz，板载算力毫无压力。
- 滚动时域：只下发第一步指令。

### 7.5 指令下发

MPC 输出速度/加速度 → `mavros/setpoint_raw/local`（`IGNORE_AFX/AFY/AFZ` + velocity + yaw），yaw = `atan2(目标相对位置)` 让机头朝目标。

---

## 8. 状态机与安全

在现有 [fly_px4.cpp](src/fly_px4.cpp) 状态机基础上，**仅替换追踪段**：

```
TAKEOFF → FLYING → TRACKING(MPC) → LANDING
```

安全兜底：

- 目标丢失 → 悬停 / 返航
- 速度饱和、急停
- 地理围栏
- 掉高检测（保留现有逻辑，但去掉与速度指令冲突的问题）

---

## 9. 分阶段实施路线

| 阶段 | 内容 | 交付 |
|---|---|---|
| 0 | 坐标系与消息约定，NED/ENU 单测 | 转换函数 + 单测 |
| 1 | 单目 AprilTag 检测 + PnP + TF | 相机节点 |
| 2 | target_tracker（KF + TargetState 消息） | 跟踪器节点 |
| 3 | MPC 控制器 + 指令下发 | 控制节点 |
| 4 | 集成状态机 + 安全兜底 | 完整流程 |
| 5 | 仿真（Gazebo/PX4 SITL + 移动目标）→ 真机调参 | 闭环验证 |

---

## 10. 风险与注意点

1. **坐标系/量纲**：NED↔ENU、机体↔局部、像素↔米，最易出 bug，阶段 0 写死单测。
2. **FOV 锥约束非凸**：需线性化或软约束，否则 QP 不收敛。
3. **模型失配**：PX4 速度环带宽有限，MPC 指令变化率要限制，避免激振。
4. **目标急转**：匀速假设失效，升级 IMM 或加不确定度惩罚。
5. **端到端延迟**：检测→KF→MPC 延迟要量化补偿。
6. **时间戳对齐**：图像、无人机位姿、KF、MPC 用统一时钟源。

---

## 11. 结论

单目 AprilTag + KF + MPC 是**可行且明显优于现有像素 PID** 的地面动态目标追踪方案：

- 单目 + 已知尺寸标记，直接解 3D 位姿（含目标航向），消除尺度歧义；
- KF 提供平滑位置、速度估计、野值剔除与丢帧桥接；
- MPC 提供预测、硬约束、FOV 约束与航向协同。

核心工作量在「AprilTag 检测 + 坐标变换 + KF」这条感知估计链，MPC 控制本身是成熟、低算力成本的部分。
