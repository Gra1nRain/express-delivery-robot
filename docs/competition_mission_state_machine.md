# 室内比赛状态机设计与离线实现

## 已实现事实

主状态链为：

```text
WAIT_START_FLAG
→ RUN_TO_TRAFFIC_STOP
→ WAIT_TRAFFIC_LIGHT
→ RUN_TO_PICKUP_FRONT
→ PICKUP_FRONT_TASK
→ [失败时 RUN_TO_PICKUP_REAR → PICKUP_REAR_TASK]
→ [有货时 RUN_TO_DROP_FRONT → DROP_FRONT_TASK]
→ [前点失败时 RUN_TO_DROP_REAR → DROP_REAR_TASK]
→ [无货时 BYPASS_DROP_TASKS]
→ RUN_TO_FINISH / BYPASS_DROP_TASKS
→ FINISHED
```

- 挥旗事件只在 `WAIT_START_FLAG` 有效，收到后启动路线。
- 距红绿灯停车点 `1.0 m` 时发布非停车标记
  `traffic_light_vision_on`，仅开启算法，不改变规划或避障。
- 红绿灯点满足位置、航向、速度和稳定时间后进入等待。稳定绿灯立即放行；
  `RED/YELLOW` 重置连续无结果计时；`UNKNOWN/OFF` 连续 `15 s` 后降级放行。
- `PICKUP` 先识别图片并锁定目标类型，再识别实物、抓取并确认持物。只有带目标类型
  的确认成功结果会设置 `has_cargo=true`。
- 前点任何非成功结果都前往后点；后点最终失败会放弃本环节并继续。
- 装货最终失败会进入 `BYPASS_DROP_TASKS`，经过卸货区但不停车。
- 控制器停稳后进入 `WAIT_RELEASE` 并持续输出零命令，不再按固定时间自动推进。
  总状态机通过 `/mission/checkpoint_release` 指定下一个停车点，可跳过中间检查点。
- 避障仍由规划器全程管理；状态机不启停、不调参。

## ROS 接口

| 方向 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 输入 | `/perception/flag_wave_detected` | `std_msgs/Bool` | 新挥旗事件 |
| 输入 | `/mission/marker_passed` | `std_msgs/String` | 非停车语义标记 |
| 输入 | `/control/status` | `std_msgs/String` JSON | `WAIT_RELEASE`、当前停车点等 |
| 输入 | `/perception/traffic_light_detection` | `std_msgs/String` JSON | 当前原始灯色，用于无结果计时 |
| 输入 | `/perception/traffic_light_state` | `std_msgs/String` | 已稳定确认的绿灯状态 |
| 输出 | `/mission/route_enable` | `std_msgs/Bool` | 挥旗后的路线总使能 |
| 输出 | `/mission/checkpoint_release` | `std_msgs/String` | 显式选择并放行下一停车点 |
| 输出 | `/perception/traffic_light_enable` | `std_msgs/Bool` | 按阶段启停红绿灯推理 |
| 输出 | `/mission/status` | `std_msgs/String` JSON | 当前状态、货物和机械臂任务 |
| 双向 | `/mission/arm_task` | `competition_interfaces/action/ArmTask` | 常驻机械臂任务 |

`ArmTask` 的 `PICKUP` feedback 明确包含图片识别、目标类型锁定、实物搜索、操作和
确认阶段。前点已经锁定的 `target_type` 会作为后点 goal hint 复用。

## 配置与离线证据

- 状态机参数：`config/mission/indoor_competition_mission.yaml`
- 语义路线：`config/routes/indoor_competition_mission_route.yaml`
- 来源一致的轨迹：
  `docs/evidence/day5/indoor_competition_mission_trajectory.yaml`
- 轨迹事实报告：
  `docs/evidence/day5/indoor_competition_mission_trajectory_summary.md`
- 入口：`ros2 launch competition_bringup indoor_competition.launch.py`

## 未验证与风险

- 真实 Piper 程序仍是用户未跟踪的迁移源码，当前没有结构化任务结果；本次没有修改
  该目录。`ArmTask` 的真实常驻适配器尚未完成，现有模拟适配器不会发送机械臂命令。
- 当前抓取代码尚未把夹爪角度/力反馈接入可靠的持物确认；Action server 必须完成该项
  后才可返回 `PICKUP SUCCESS`。
- `1.0 m` 红绿灯预触发距离和 `120/90 s` 机械臂总超时是初始配置，需在无底盘运动的
  单模块计时后复核。
- 按已确认范围，第一版没有实现挥旗永久失败和导航不可达恢复。
- 所有验证均为电脑端离线验证，不代表 ROS 2 车端构建、真实视觉、机械臂或底盘运动
  已通过。
