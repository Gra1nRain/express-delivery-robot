# Day 3 实车低速试跑补充记录

记录时间：2026-07-22

## 范围

事实：

- 本次试跑是 Day3 规划链路后的额外实车验证，不属于原计划表中 Day3 的核心验收项。
- 本次没有启动机械臂，没有发送取货/卸货动作。
- 本次实车控制入口为 `scripts/day3_follow_global_plan.py`，发布 `/cmd_vel`。
- 试跑前已修复执行坐标链为 `map -> odom -> base_link`，不再使用 `map -> camera_init -> body` 作为底盘控制位姿。

## 证据文件

- `day3_navigation_launch_20260722.txt`：Day3 导航链路启动日志。
- `day3_tf_tree_after_initialpose_20260722.txt`：AMCL 初始位姿后 TF 树采样。
- `day3_global_path_follower_trial_20260722.txt`：低速路径跟踪器试跑日志。

## 试跑前检查

事实：

- `/cmd_vel` 在启动跟踪器前无发布者，`ranger_base_node` 为唯一订阅者。
- `/system_state`：`vehicle_state=0`、`control_mode=1`、`error_code=0`、`battery_voltage≈49.5V`。
- `/scan.header.frame_id=base_link`。
- TF 树已闭合为 `map -> odom -> base_link -> livox_frame/camera_link`。
- AMCL 初始位姿使用 `semantic_map.yaml` 中 `start=(-0.376, 0.112, yaw=0.020)`。
- 初始 `map -> base_link` 复核约为 `(-0.379, 0.111, yaw=0.021)`，起点误差约 `0.003m`。

## 试跑结果

事实：

- 第一次启动后，跟踪器因短暂 TF 时间外推异常停止；停止后 `/cmd_vel` 无发布者，`/odom` 速度为 0。
- 随后补丁 `9bc95c1 Tolerate transient TF dropouts` 增加短时 TF 抖动容忍：短暂 TF 异常先发零速等待，连续超过阈值才退出。
- 第二次从当前位置继续试跑，起点容差临时放宽到 `0.80m`，因为车辆已从首点前进到路径 index 3 附近。
- 试跑过程中车辆沿路径前进到约 index 6；日志中最大可见路径误差约 `0.35m`。
- 最终位姿约 `map -> base_link = (x=1.000, y=0.368, yaw=1.222rad)`。
- 最终最近路径点为 index `6/220`，最终路径偏差约 `0.200m`。
- Codex 主动停止跟踪器；停车后 `/cmd_vel` 无发布者、`/odom` twist 为 0、底盘 `error_code=0`。

## 判断

事实：

- 新坐标链和底盘 `/cmd_vel` 通路已经实车验证可用。
- 当前 AMCL/TF 航向在低速运动中仍有明显跳变；试跑日志中 yaw 多次大幅变化，导致简单全局路径跟踪器反复修正方向。

未通过：

- “全程稳定自动跟踪”未通过。
- 当前简单跟踪器不能作为 Day5 轨迹跟踪最终方案。

建议：

- 不继续硬跑全程。
- 下一步优先稳定定位/航向来源，或接入之前跑通过的 Nav2 controller/local planner。
- Day4 应把全局 Path 下游改成带速度、曲率和时间戳的轨迹，再进入 Day5 跟踪误差验收。
