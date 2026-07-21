# competition_planning

负责目标点选择、确定性语义走廊全局路径、后续 Hybrid A*/State Lattice 插槽和局部轨迹优化。规划结果必须显式发布，不能直接绕过轨迹跟踪器控制底盘。

Day3 当前先交付确定性语义走廊规划：

- `semantic_planner.py`：纯 Python 规划与合法性校验接口。
- `offline_global_plan`：离线读取 route、semantic map、planning params，输出每个可规划 step 的路径或失败原因。
- `semantic_global_path_node`：复用同一规划结果，按当前 `step_id` 发布一条 `nav_msgs/Path` 到 `/planning/global_path`。

本模块不输出速度命令，不控制底盘，不触发机械臂动作。
