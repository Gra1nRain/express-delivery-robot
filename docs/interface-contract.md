# ROS2 接口契约草案

本文只记录当前方案中的接口意图。消息字段、QoS、frame_id、超时和错误码在接口联调前必须由相关成员确认并冻结。

## 规划控制

| 接口 | 方向 | 说明 |
|---|---|---|
| `/planning/corridor` | mapping -> planning | 当前语义走廊 |
| `/planning/global_path` | planner -> optimizer | `nav_msgs/Path` 候选 |
| `/planning/optimized_trajectory` | optimizer -> tracker | 带速度、曲率和时间的轨迹 |
| `/control/body_cmd` | tracker -> safety | `geometry_msgs/TwistStamped`；`linear.x` 为 m/s，`angular.z` 为期望 yaw rate rad/s |
| `/control/tracking_error` | tracker -> safety/logger | `Vector3Stamped` 临时 adapter：x=横向误差 m，y=航向误差 rad，z=target index |
| `/control/state_valid` | state estimator -> safety | TF/odom 新鲜且连续时为 true |
| `/control/status` | tracker -> safety/logger | 临时 JSON adapter；至少含 `status`、`target_index`、`state_reasons` |
| `/control/wheel_cmd` | kinematics -> adapter | 四轮调试命令，是否落地待验证 |
| `/cmd_vel_safe` | safety -> chassis | 最终安全速度出口 |

`/planning/optimized_trajectory` 的 Day5 连续 artifact 字段为：`x`、`y`、`yaw`、`s`、`curvature`、`v`、`a`、`jerk`、`yaw_rate`、`t`、`ref_id`。顶层 `source_manifest` 保存路线、语义图、规划/优化参数、栅格地图 YAML 和栅格图像的 SHA-256；控制节点启动前逐项复算，不一致即拒绝运行。正式 ROS2 msg 尚未冻结；控制节点当前从 YAML 加载冻结轨迹，避免在线规划抖动直接进入底盘闭环。

定位唯一权威链为锚定 FAST-LIO 的 `map -> camera_init -> body`；`/odom` 只提供 RANGER 速度反馈，不发布竞争性的 `map -> odom`。Day5 launch 不启动 AMCL。

最终输出 `/cmd_vel_safe` 由 `competition_safety` 独占。实车驱动实际订阅 `/cmd_vel` 时，只允许把 safety 的 `command_output_topic` 显式设为 `/cmd_vel`，控制节点不得直接 remap 到该 topic。

## 任务与作业

| 接口 | 方向 | 说明 |
|---|---|---|
| `/mission/state` | mission -> all | 当前状态和货物状态 |
| `/mission/events` | mission -> logger | step、异常和恢复记录 |
| `/dock` | mission -> docking | 取货、投放、终点精停 action |
| `/arm/logistics_task` | mission -> arm bridge | 机械臂取放 action，字段待确认 |
| `/dock/status` | docking -> mission | 精停误差和稳定状态 |

## 避障与安全

| 接口 | 方向 | 说明 |
|---|---|---|
| `/avoidance/status` | avoidance -> planning/safety | active、mode、confidence、speed_limit |
| `/avoidance/objects` | avoidance -> planning | 障碍物/行人/锥桶结果 |
| `/avoidance/corridor_update` | avoidance -> planning | 带有效期的临时走廊 |
| `/avoidance/stop_request` | avoidance -> safety | 立即停车请求 |
| `/safety/event` | safety -> mission/logger | 安全停车和故障原因 |

## 冻结规则

1. 先用最小字段跑通主链路。
2. 字段、单位、坐标系、有效期和错误处理必须写进本文件。
3. 接口变更必须同步更新测试矩阵和当天日志。
4. 避障组未接入前，主控使用空模块或 mock publisher。
5. 避障算法可由避障组调研，但部署适配、输入输出验收和本车验证由主控侧负责冻结。
6. `Vector3Stamped` 和 JSON status 是 Day5 临时 adapter，不等于自定义接口已冻结；替换时必须保持 SafetySupervisor 的纯 Python interface 测试不变。
