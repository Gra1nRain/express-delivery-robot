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

## 车端无运动验证（2026-07-22）

- 小车 `/home/agilex/competition_ws` 已成功构建 `competition_planning`、`competition_localization`、`competition_control`、`competition_safety` 和 `competition_bringup`，5 个包总构建时间 `14.8s`。
- 小车安装环境重跑整线离线闭环通过：4596 周期、最大横向误差 `0.060m`、0 次 `SAFE_HOLD`；远程命令总耗时约 `99s`，折算平均每周期约 `21.5ms`，低于 20Hz 的 `50ms` 周期预算。该数值仍不是 ROS timer 的实测抖动。
- 在 `start_livox:=false`、`start_fast_lio:=false`、`start_base:=false`、`command_output_topic:=/cmd_vel_safe` 下，`fastlio_anchor`、`mppi_control` 和 `competition_safety` 均成功启动；`/cmd_vel` 不存在，`/cmd_vel_safe` 实测全零。
- 无传感器和底盘输入时，controller 正确报告 `INVALID_STATE`，safety 正确报告 `SAFE_HOLD`，没有非零速度出口。
- 首次 smoke test 暴露的 shutdown `RCLError` 已在提交 `c7ae1d2` 修复；车端 shutdown guard 回归测试 `2 passed`，节点关闭后无残留进程。
- 车端日志：`log/day5_build_e1c2ff5.log`、`log/day5_car_offline_validation_e1c2ff5.log`、`log/day5_nomotion_smoke_e1c2ff5.log` 和 `log/day5_shutdown_repro_c7ae1d2.log`。
- 实车位于起点后启动 Livox、FAST-LIO、Ranger base、anchor、MPPI 和 safety，且 `command_output_topic:=/cmd_vel_safe` 时，`/cmd_vel` 只有 Ranger 订阅、发布者为 0；底盘 `/odom` 速度保持全零。
- 首次硬件无运动联调暴露 FAST-LIO 输出延迟：`/Odometry` 与 `camera_init -> body` TF 的 stamp age 稳定约 `1.3-1.6s`，控制器正确进入 `INVALID_STATE` / `stale_pose`，safety 保持 `SAFE_HOLD`。
- 提交 `cca6797` 新增 `fast_lio_mid360_day5_control.yaml`，关闭 Day5 运动控制不需要的 FAST-LIO path/map/scan/cloud/PCD 输出；Day5 motion launch 默认使用该控制配置，Day1 建图配置保持不变。
- 使用 `cca6797` 后，FAST-LIO-only `/Odometry` stamp age 降至约 `0.037-0.068s`；完整硬件无运动栈下 `/Odometry` stamp age 为 `0.037-0.128s`，低于 `pose_timeout_s=0.20`。
- 发布起点 `/initialpose`（`x=-0.376, y=0.112, yaw=0.020rad`）后，控制器稳定 `TRACKING`、`state_valid=true`，safety 稳定 `SAFE_ACTIVE`；`/control/status` 约 `19.8-20.4Hz`。
- 锚定后 `/cmd_vel_safe` 出现约 `0.01m/s` 的安全后起步命令，说明规控链已具备运动输出；该 topic 没有底盘订阅者，因此本次仍属于无运动联调。
- 追加车端日志：`log/day5_bringup_build_cca6797.log`、`log/day5_fastlio_only_baseline_961a12e.log`、`log/day5_fastlio_control_profile_cca6797.log` 和 `log/day5_hw_nomotion_cca6797.log`。

## 短段实车验证（2026-07-22）

- 现场确认急停有效、遥控器可接管、车前方 3m 安全后，将 safety 输出接入 `/cmd_vel`，执行 `debug_traffic_light_trajectory.yaml` 短段实车。
- 启动前 `/cmd_vel` 拓扑为：`competition_safety` 一个发布者，`ranger_base_node` 和 `rosbag2_recorder` 两个订阅者；发布 `/initialpose` 前 `/cmd_vel` 和 `/odom` 均为零。
- 监控脚本发布起点 `/initialpose` 前检测到 1 个订阅者；发布后约 `1.006s` 首次进入 `TRACKING`，约 `21.573s` 首次进入 `GOAL_REACHED`。
- 运行最大 `/cmd_vel.linear.x` 为 `0.1979m/s`，最大 `/odom.twist.twist.linear.x` 为 `0.1990m/s`；最终 `/cmd_vel` 与 `/odom` 速度均回到 0。
- 最终 tracking error 为横向 `0.013m`、航向 `0.016rad`，target index `26`；短段完成后 safety 进入 `SAFE_STOP`。
- 本次短段实车录包保存在小车 `/home/agilex/competition_ws/recordings/day5_short_traffic_804336e_20260722_191755`，大小 `9.6MiB`；证据摘要见 `docs/evidence/day5/day5_short_traffic_field_trial_20260722.md`。
- rosbag metadata 中 `/initialpose` 计数为 0；该发布事件由小车 `log/day5_short_motion_monitor_804336e.txt` 记录。
- 停止 launch 和 rosbag 后，小车端无 Day5、Livox、FAST-LIO、Ranger、MPPI、safety 或 rosbag 残留进程，ROS 主题回到 `/parameter_events` 和 `/rosout`。

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

3. 无运动启动短段，确认 `/cmd_vel` 无 Day5 publisher，发布 `/initialpose` 后检查 TF/odom/system/motion/status；锚定后 `/cmd_vel_safe` 可以出现规控输出，但它不能有底盘订阅者：

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

- Day5 已完成 Livox、FAST-LIO、Ranger base、anchor、MPPI 和 safety 的联合无运动检查，并完成 `traffic_light_stop_line` 2.6m 短段实车；尚未执行 `drop_dock` 半程或整线实车，本记录不声称整线实车已通过。
- 离线模型没有包含执行器延迟、轮胎侧偏、地面摩擦变化和 FAST-LIO 实际噪声。
- 曲率连续非线性优化、CBF/QP、显式差速自旋恢复和连续 footprint 扫掠仍在 `docs/algorithm-debt.md` 登记。
