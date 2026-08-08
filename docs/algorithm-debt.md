# 算法债务登记

本表防止诊断算法、临时 adapter 或低速约束在后续被误当成最终方案。`OPEN` 项未关闭时，不得把对应能力写成正式通过；`FROZEN` 表示实验室功能基线已经由现场操作员接受，比赛前不做高风险结构重构，但不等同于正式赛场证据验收。

| 状态 | 能力 | 正式目标 | 当前实现 | 替换触发与最晚期限 | 正式验收限制 |
|---|---|---|---|---|---|
| CLOSED | 全局规划 | Hybrid A*，约束 x/y/yaw 与最小转弯半径 | `HybridAStarPlanner` 已为默认；`0.81m` 半径进入配置和测试 | 2026-07-22 已关闭；实车参数变化时重开 | occupancy-grid A* 只能作连通性诊断 |
| CLOSED | 局部跟踪 | MPPI | 768 rollouts、30 步 horizon、RANGER 曲率模型、进度跳变保护 | 2026-07-22 离线关闭；车端 20Hz 不达标时重开 | 不得用 RPP 结果替代 MPPI 正式通过 |
| FROZEN | 在线局部避障 / 代价图 | 基于最新二维 Scan 和膨胀代价图的局部 Hybrid A* 绕障，并在瞬时规划超时时保持上一条经碰撞检查的安全轨迹 | 已部署为 body 点云 -> 2D Scan -> `0.44m` 膨胀 costmap -> `reference_aware_hybrid_astar` -> MPPI；距离 hard-stop 按现场决定禁用，避障输入过期、局部规划失败/轨迹过期和车辆状态无效仍会停车。实验室整圈已跑通并冻结为 `day5-avoidance-lab-ready-v1` | 正式赛场适配、速度提高或出现可复现实车回归时重开；机械臂阶段不得顺手调参 | 当前只认实验室功能基线；连续三次、最新 rosbag/视频和正式赛场证据仍未完成 |
| FROZEN | 避障接口边界 | 避障算法长期应通过 `config/avoidance/` 约定接口与主控解耦 | 为满足比赛期低延迟联调，局部 Hybrid A* 和连续性策略当前直接集成在 `competition_planning`；这是用户批准的阶段性架构例外 | 比赛前不迁移、不拆分 `LocalTrajectoryPlanner.plan()`；赛后或正式接口冻结后再迁移 | 机械臂、精停和任务状态机不得继续向局部规划器堆叠职责 |
| OPEN | 几何优化 | 曲率连续、曲率变化率受约束的非线性轨迹优化 | 9 档曲率 Hybrid A*、语义锚点零曲率边界、轨迹 `0.80 1/m/s` 硬校验；MPPI 对命令曲率变化率限幅 | 提速超过 `0.20m/s` 或进入正式比赛场地前，最晚 Day6 调速前 | Day5 低速轨迹已满足曲率变化率包络；尚不得宣称高速度非线性轨迹优化已完成 |
| OPEN | 安全监督 | CBF/QP 基于障碍距离和车辆动力学修正命令 | 独立 hard-rule SafetySupervisor；当前保留状态、时效、控制权和运动模式门控，不启用距离 hard-stop | 正式赛场障碍接口稳定后再评估，不阻塞当前低速实验室基线 | 现有门控是必须出口，但不能宣称 CBF 碰撞可行域已验证 |
| OPEN | 差速自旋恢复 | 带滞回、速度归零和显式状态的混合运动恢复 | 正常跟踪禁用自旋；检测到非双阿克曼立即 SAFE_HOLD | Day6 状态机加入恢复状态时实现 | Day5 三次整线中一旦触发自旋即失败，不得算自动恢复成功 |
| OPEN | 诊断 tracker | RPP 仅作可复现实验对照 | 尚未实现；配置明确为 `not_implemented` | MPPI 实车基线通过后再补，最晚首次控制器 A/B 报告前 | 不存在自动 fallback，不能因 MPPI 故障静默切换 |
| OPEN | ROS 自定义接口 | 冻结 trajectory/body/safety status msg | Day5 用 YAML、TwistStamped、Vector3Stamped 和 JSON adapter | 字段稳定并完成车端联调后，最晚 Day8 总任务闭环前 | 临时 adapter 不是已冻结接口 |
| OPEN | 连续 footprint 检查 | 车辆矩形/扫掠体碰撞与 clearance 约束 | 已新增离线矩形 footprint sweep checker；控制验证轨迹要求 `0.72x0.50m + 0.20m` clearance 通过。规划器内部仍是中心点按 `0.30m` 膨胀检查 | 正式场地图冻结或速度提高前，最晚 Day6 场地复核 | checker 已能拦截贴边轨迹，但尚未并入 Hybrid A* 搜索代价/约束 |

## 历史算法身份

- `scripts/day3_follow_global_plan.py`：历史低速调试脚本，不是正式 tracker。
- `occupancy_grid_astar`：连通性/地图诊断 backend，不是当前正式全局规划器。
- `cubic_bezier`：Day3/Day4 诊断平滑器；其高曲率输出不得进入 Day5 正式实车轨迹。
- `semantic_corridor`：配置与 route 合法性测试 adapter，不是可直接上车的正式规划器。
- `planning_params.yaml` 中的 `replanning.plugin: dwa`：冻结轨迹 source manifest 的遗留 provenance 字段，当前 launch 不用它选择局部算法；运行时唯一算法由 `local_hybrid_astar_runtime_params_day5.yaml` 的 `reference_aware_hybrid_astar` 决定。修改该遗留字段前必须同步重生成并复核轨迹 artifact，比赛期禁止只为改名而改动。
