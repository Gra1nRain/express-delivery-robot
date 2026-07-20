# 项目架构草案

## 主链路

```text
传感器 / FAST-LIO / D435
        ↓
语义地图与路线 step
        ↓
目标点选择
        ↓
全局路径规划
        ↓
局部轨迹优化
        ↓
轨迹跟踪
        ↓
四轮运动学 / 底盘适配
        ↓
安全层
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
| `competition_planning` | 目标选择、Hybrid A*/State Lattice、轨迹优化 |
| `competition_control` | tracker、运动模式、四轮运动学和底盘适配 |
| `competition_perception` | 挥旗、红绿灯、停车牌接口 |
| `competition_mission` | P0 状态机、dock、机械臂桥接和任务日志 |
| `competition_safety` | 急停、断流、越界、超速和避障停车兜底 |
| `competition_avoidance_interface` | 避障组接口适配，不实现避障组算法 |

## 场地配置原则

`debug_site_profile.yaml` 和 `competition_site_profile.yaml` 是两套独立入口。路线 step 使用官方语义命名；实验室场地物理复用不能改变 step 语义。正式场地适配只替换地图、路线点、ROI、dock pose、限速和小范围阈值。

## 传感器命名

D435 必须按硬件身份和稳定 USB 路径绑定命名空间，不能依赖 `/dev/video*` 编号。当前小车端 `realsense2_camera` 用 `usb_port_id` 选择设备；序列号用于记录硬件身份。

| 名称 | 位置 | 序列号 | 当前 ROS 选择器 | 主要用途 |
|---|---|---:|---|---|
| `front_camera` | 车体前向 | `236223021647` | `usb_port_id=2-3.1.1.1` | 挥旗、红绿灯、停车牌 |
| `left_wrist_camera` | 机械臂腕部 | `152223024925` | `usb_port_id=2-3.3.2` | 取放货位姿、手眼感知 |

## 未验证项

- RANGER 驱动是否支持四轮转角/轮速接口。
- `/cmd_vel.linear.y` 是否在实际底盘生效。
- 自定义消息字段与机械臂实际 action 协议。
- 两台 D435 的 color 与 aligned depth topic 已验证；frame_id、点云 topic 和 USB 带宽余量仍需正式验证。
