# 项目架构草案

## 主链路

```text
传感器 / FAST-LIO / D435
        ↓
语义地图与路线 step
        ↓
目标点选择
        ↓
连续整线 Hybrid A* 全局路径
        ↓
jerk-limited S 曲线局部轨迹优化
        ↓
实时 costmap + 参考轨迹约束的短视距 Hybrid A* 重规划
        ↓
Day4 速度/曲率/时间参数化
        ↓
MPPI 轨迹跟踪（RANGER 曲率模型）
        ↓
SafetySupervisor（硬规则，CBF/QP 待演进）
        ↓
/cmd_vel_safe 或厂家底盘接口
```

## 代码包职责

| 包 | 职责 |
|---|---|
| `competition_interfaces` | 自定义 msg/action/srv；字段冻结前只允许草案 |
| `competition_bringup` | 总启动入口和模块启动编排 |
| `competition_localization` | FAST-LIO/全局坐标锚定等定位适配 |
| `competition_mapping` | 语义地图、路线图和规划走廊 |
| `competition_planning` | 语义目标、9 档曲率 Hybrid A*、整线连续路径、曲率变化率硬校验和 jerk-limited 时间参数化；栅格 A*/Bezier 仅作历史诊断 |
| `competition_control` | MPPI tracker、四轮四转曲率模型、进度管理和 ROS body command adapter |
| `competition_perception` | 挥旗、红绿灯、停车牌接口 |
| `competition_mission` | P0 状态机、dock、机械臂桥接和任务日志 |
| `competition_safety` | 独占最终速度出口；检查急停/CAN 控制权/断流/误差/底盘模式，并施加速度、加速度和曲率硬约束 |
| `competition_avoidance_interface` | 避障输入输出接口适配；避障组可调研算法，主控侧负责本车部署适配和验证 |

## 场地配置原则

`debug_site_profile.yaml` 和 `competition_site_profile.yaml` 是两套独立入口。路线 step 使用官方语义命名；实验室场地物理复用不能改变 step 语义。正式场地适配只替换地图、路线点、ROI、dock pose、限速和小范围阈值。

## 传感器命名

D435 必须按硬件身份和稳定 USB 路径绑定命名空间，不能依赖 `/dev/video*` 编号。当前小车端 `realsense2_camera` 用 `usb_port_id` 选择设备；序列号用于记录硬件身份。

| 名称 | 位置 | 序列号 | 当前 ROS 选择器 | 主要用途 |
|---|---|---:|---|---|
| `front_camera` | 车体前向 | `236223021647` | `usb_port_id=2-3.1.1.1` | 挥旗、红绿灯、停车牌 |
| `left_wrist_camera` | 机械臂腕部 | `152223024925` | `usb_port_id=2-3.3.2` | 取放货位姿、手眼感知 |

## 未验证项

- Day5 MPPI 在小车 CPU 上以 20 Hz、768 rollouts 运行的实际周期与抖动。
- `map -> camera_init -> body` 锚定 FAST-LIO 与 `/odom` 速度并用时的实车时间同步误差。
- 曲率连续非线性优化器和 CBF/QP 安全监督，见 `docs/algorithm-debt.md`。
- `/cmd_vel.linear.y` 是否在实际底盘生效；Day5 正常跟踪不使用横移。
- 自定义消息字段与机械臂实际 action 协议。
- 两台 D435 的 color 与 aligned depth topic 已验证；frame_id、点云 topic 和 USB 带宽余量仍需正式验证。

## RANGER MINI 3.0 运动事实

- 用户手册第 4–5 页给出轴距 `0.494m`、前/后轮距 `0.364m`、阿克曼最小转弯半径 `0.810m`、四轮四转和最高速度 `7.2km/h`。
- 用户手册第 14–15 页给出控制帧线速度上限 `2.0m/s`、内轮转角上限约 `0.698rad`；转角超过 20° 时协议速度范围降为 `0.7m/s`。
- 车端 `ranger_messenger.cpp` 已只读核对：`Twist.angular.z` 先以 `R=|v/ω|` 换算转角；半径低于驱动阈值会切到自旋模式并把线速度置零。
- Day5 正常跟踪只允许双阿克曼模式。四轮差速自旋作为显式恢复能力保留，不允许在整线验收中隐式触发。
- Day5 Hybrid A* 以零曲率穿越语义锚点，轨迹生成阶段硬校验曲率变化率不超过 `0.80 1/m/s`；当前 debug 整线实测最大值为 `0.309 1/m/s`。
