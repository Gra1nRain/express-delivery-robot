# Ranger Mini V3 官方车辆模型审计（2026-07-24）

## 范围

本记录来自小车 `/home/agilex/agilex_ws` 的只读核对，没有启动 ROS 节点或发送运动指令。

核对版本：

- `ranger_ros2`: `b6ea21a275ca5e7168130cc6470e61474681d679`，分支 `humble`
- `ugv_sdk`: `c3dfaf444f9bae10757e546acae055aaf4a13de7`，分支 `main`
- 上游仓库：
  - <https://github.com/agilexrobotics/ranger_ros2>
  - <https://github.com/agilexrobotics/ugv_sdk>

小车的 `ranger_ros2` 有两处既有本地修改：移除自定义 SIGINT 处理、给 V3 bringup 补传 `update_rate`。本次没有修改这些文件；两处修改均未改变运动学公式。

## 已确认事实

### 官方几何和限制

`ranger_base/include/ranger_base/ranger_params.hpp` 的 `RangerMiniV3Params` 定义：

- 轮距：`track = 0.364 m`
- 轴距：`wheelbase = 0.494 m`
- 最大线速度：`1.5 m/s`
- 最大角速度：`4.8 rad/s`
- dual-Ackermann 最大转角：`0.6981 rad`
- 驱动最小转弯命令半径：`0.47644 m`

### `/cmd_vel` 的实际接口语义

`ranger_base/src/ranger_messenger.cpp` 在 `linear.y == 0` 时：

1. 以 `R_cmd = |linear.x| / |angular.z|` 计算命令半径。
2. 以 `phi_inner = atan((wheelbase / 2) / R_cmd)` 计算内轮转角。
3. `R_cmd < 0.47644 m` 时切换到 spinning mode，否则使用 dual-Ackermann。
4. dual-Ackermann 模式最终向底盘发送线速度和转角，而不是直接发送车体 yaw-rate。

因此，Ranger 驱动的 `/cmd_vel.angular.z` 在 dual-Ackermann 模式下是“用于反推转角的半径编码字段”，不能直接等同于最终车体 yaw-rate。

### 官方 dual-Ackermann 状态模型

`ranger_base/include/ranger_base/kinematics_model.hpp` 使用：

```text
x_dot     = v * cos(phi_center) * cos(theta)
y_dot     = v * cos(phi_center) * sin(theta)
theta_dot = 2 * v * sin(phi_center) / wheelbase
```

内轮角到中心角的换算为：

```text
phi_center = atan(
  wheelbase * sin(phi_inner)
  / (wheelbase * cos(phi_inner) + track * sin(phi_inner))
)
```

把官方 `R_cmd = 0.47644 m` 代入上述模型，得到最小非 spinning 的车体轨迹半径约 `0.703 m`。因此 `0.47644 m` 不是规划器应直接采用的车体最小转弯半径。

### odom 与 TF

官方驱动：

- 始终发布 `/odom`，默认 frame 语义为 `odom -> base_link`。
- dual-Ackermann odom yaw-rate 使用上述中心转角模型计算。
- 仅在 `publish_odom_tf=true` 时广播 `odom -> base_link` TF。
- 当前源码参数默认值和 Ranger Mini V3 launch 默认值均为 `publish_odom_tf=false`；仓库 README 写 `true`，两者不一致，应以实际源码和 launch 为准。

## 对当前比赛代码的判断

### 已验证正确

- `RangerMiniV3Geometry(wheelbase=0.494, track_width=0.364)` 与官方包一致。
- `ranger_twist_adapter` 对官方 `/cmd_vel` 半径语义做逆映射，方向正确。
- 实车 adapter 试验得到约 `0.812 m` 车体半径，与 `0.81 m` 目标一致，进一步验证了接口适配。
- 规划/控制使用 `0.81 m` 作为保守车体最小半径是合理的；不能用驱动的 `0.47644 m` 直接替代。

### 仍需统一

当前 MPPI 和 Hybrid A* 使用通用曲率模型，尚未显式包含官方模型中的 `cos(phi_center)` 平移项，也没有共享同一套：

- 车体曲率与中心转角换算
- 车体 yaw-rate 与 Ranger `/cmd_vel.angular.z` 换算
- 可执行曲率上限
- 曲率变化率/转向执行器限制
- dual-Ackermann、parallel、spinning 模式策略

## 建议

### Day5

- 保留 `0.81 m` 低速 dual-Ackermann 约束和现有 adapter。
- Day5 的权威定位链继续使用 `map -> camera_init -> body`。
- `/odom` 继续作为速度消息来源。
- Day5 禁止 Ranger 广播孤立的 `odom -> base_link` TF；这与官方 V3 launch 默认行为一致。
- 不在未标定 `body` 与底盘几何中心关系前伪造 `body -> base_link`。

### 后续车辆模型 module

建立一个小接口、深实现的 Ranger 车辆模型 module，让 global planner、local planner、MPPI、safety 和 adapter 共用：

```text
body_curvature <-> center_steering_angle
body_yaw_rate  <-> ranger_driver_twist
step(state, command, dt)
feasible(command, speed)
```

实现应直接复用本记录确认的官方公式；adapter 作为该 interface 的 Ranger ROS 驱动侧 adapter，而不是让几何常量继续散落在多个调用方。

## 未验证项

- `body` 到真实底盘几何中心/`base_link` 的外参尚未完成标定。
- 官方最大转角和 `0.47644 m` spinning 阈值之间的设计意图未在源码注释中解释。
- 当前 MPPI 通用模型与官方 dual-Ackermann 模型在 Day5 速度/转角范围内的累计预测误差尚未量化。
