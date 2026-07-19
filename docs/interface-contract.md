# ROS2 接口契约草案

本文只记录当前方案中的接口意图。消息字段、QoS、frame_id、超时和错误码在接口联调前必须由相关成员确认并冻结。

## 规划控制

| 接口 | 方向 | 说明 |
|---|---|---|
| `/planning/corridor` | mapping -> planning | 当前语义走廊 |
| `/planning/global_path` | planner -> optimizer | `nav_msgs/Path` 候选 |
| `/planning/optimized_trajectory` | optimizer -> tracker | 带速度、曲率和时间的轨迹 |
| `/control/body_cmd` | tracker -> chassis | 车体速度命令 |
| `/control/wheel_cmd` | kinematics -> adapter | 四轮调试命令，是否落地待验证 |
| `/cmd_vel_safe` | safety -> chassis | 最终安全速度出口 |

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
