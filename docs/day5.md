# Day 5 整线运动规控记录

## 事实

- 正式链路已实现为：锚定 FAST-LIO 全局位姿 + RANGER `/odom` 速度 -> 连续 Hybrid A* -> jerk-limited S 曲线 -> MPPI -> SafetySupervisor -> `/cmd_vel_safe`。
- `config/planning/planning_params.yaml` 默认规划器已从 occupancy-grid A* 改为 Hybrid A*，最小转弯半径按 RANGER MINI 3.0 手册设为 `0.81m`。
- 连续控制轨迹不按任务 step 停车；起步速度为零，只有最终 `finish_park` 再次为零。
- MPPI 使用 `yaw_rate=v*curvature` 的四轮四转曲率模型；输出半径小于 `0.81m` 会被 controller 和 safety 两层限制。
- 状态连续性守卫会拒绝 TF/pose 超时、时间倒退、位置跳变和航向跳变；无效观测不会替换上一有效状态。
- safety 节点从真实 `/system_state` 判断急停、错误码和 CAN 控制权，从 `/motion_state` 检测是否意外进入自旋/斜移模式。
- Hybrid A* 使用 9 个曲率档位并以零曲率穿越语义锚点；轨迹生成阶段硬拒绝超过 `0.80 1/m/s` 的曲率变化率。
- 冻结轨迹保存路线、语义图、规划/优化配置、栅格 YAML 和图像的 SHA-256；控制节点启动前逐项校验，防止旧轨迹配新地图或新参数。
- 分段验证轨迹必须在规划阶段用 `--end-ref` 重新执行 jerk-limited 参数化；控制器不再支持运行时截断轨迹。
- launch 默认 `start_base:=false` 且 `command_output_topic:=/cmd_vel_safe`，不具备默认实车运动效果。

## 离线验证

- 连续轨迹：447 点、`44.596m`、`224.480s`；最大速度 `0.200m/s`、最大绝对加速度 `0.102m/s²`、最大绝对 jerk `0.400m/s³`、最大绝对曲率 `1.234568 1/m`、最大曲率变化率 `0.309 1/m/s`。
- 整链运动学闭环：4596 个 20Hz 周期、`229.80s`；最大横向误差 `0.060m`、最大航向误差 `0.128rad`、最大速度 `0.19971m/s`、0 次 `SAFE_HOLD`、11 个终点减速周期 `SAFE_LIMITED`、最终位置误差 `0.047m`、最终速度 0。
- 分段整链：`traffic_light_stop_line` 短段 `2.600m`、285 周期、0 次 `SAFE_HOLD`；`drop_dock` 半程 `19.799m`、2052 周期、最大横向误差 `0.044m`、0 次 `SAFE_HOLD`。
- 证据：`docs/evidence/day5/` 下的 `debug_continuous_trajectory*`、`debug_motion_validation*`、`debug_traffic_light_*` 和 `debug_drop_dock_*`。

## 无运动/实车分级步骤

1. PC 或小车生成冻结轨迹：

   ```bash
   ros2 run competition_planning offline_continuous_trajectory --route config/routes/debug_route.yaml --semantic-map maps/debug/semantic_map.yaml --planning-params config/planning/planning_params.yaml --optimizer-params config/planning/optimizer_params.yaml --output docs/evidence/day5/debug_continuous_trajectory.yaml --report docs/evidence/day5/debug_continuous_trajectory_summary.md
   ```

   短段/半程分别在同一命令中增加 `--end-ref traffic_light_stop_line` 或 `--end-ref drop_dock`，输出到 `debug_traffic_light_trajectory.yaml` 或 `debug_drop_dock_trajectory.yaml`。不能对整线 artifact 做运行时切片。

2. 小车构建：

   ```bash
   colcon build --symlink-install --packages-select competition_planning competition_localization competition_control competition_safety competition_bringup
   ```

3. 无运动启动短段，确认 `/cmd_vel` 无 Day5 publisher、`/cmd_vel_safe` 始终为零，发布 `/initialpose` 后检查 TF/odom/system/motion/status：

   ```bash
   ros2 launch competition_bringup day5_motion_control.launch.py start_base:=false command_output_topic:=/cmd_vel_safe trajectory_file:=$HOME/competition_ws/docs/evidence/day5/debug_traffic_light_trajectory.yaml
   ```

4. 只有现场确认急停、遥控接管、CAN、空场、人员、录包和视频后，才允许短段实车：

   ```bash
   ros2 launch competition_bringup day5_motion_control.launch.py start_base:=true command_output_topic:=/cmd_vel trajectory_file:=$HOME/competition_ws/docs/evidence/day5/debug_traffic_light_trajectory.yaml
   ```

5. 短段通过后，把 `trajectory_file` 改为 `debug_drop_dock_trajectory.yaml`；半程通过后再改为 `debug_continuous_trajectory.yaml` 执行整线。每次只改变一个参数并保留 rosbag/视频。

6. 整线正式通过条件：连续 3 次、最大速度 `0.20m/s`、无人接管、无 TF/pose 跳变、无意外 motion mode、无异常 safety、横向误差不超过 `0.15m`、终点可靠零速。

## 建议

- 录包至少包含 `/tf`、`/tf_static`、`/odom`、`/system_state`、`/motion_state`、`/control/body_cmd`、`/control/tracking_error`、`/control/state_valid`、`/control/status`、`/safety/event` 和最终命令 topic。
- 首次车端运行先检查 MPPI 周期；若 768 rollouts 无法稳定 20Hz，只调整 `rollout_count` 一个变量并保存周期/误差对比，不能改用 RPP 冒充正式通过。

## 未验证

- Day5 新包尚未在小车端构建或运行，本记录不声称实车已通过。
- 离线模型没有包含执行器延迟、轮胎侧偏、地面摩擦变化和 FAST-LIO 实际噪声。
- 曲率连续非线性优化、CBF/QP、显式差速自旋恢复和连续 footprint 扫掠仍在 `docs/algorithm-debt.md` 登记。
