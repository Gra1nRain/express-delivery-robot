# Day5 现代规划器调研（2026-07-24）

## 结论

对 Ranger 四轮转向底盘，不建议继续把局部 Hybrid-A* 当作“无条件重算一条到回接点的最短路”。用户要求应成为首要不变量：**局部参考段在统一 footprint、膨胀和实时代价图上可通行时，直接输出该参考段；只有障碍确实侵占参考走廊时才允许偏离。** 当前未提交工作树已经有 `path_is_navigable(local_reference) -> REFERENCE_CLEAR` 快路径，应保留并补回归测试。

推荐顺序是：保留上述快路径；障碍触发后借鉴 Nav2 MPPI 的 critic 分解；全局/自由空间规划优先评估 Nav2 Smac Hybrid-A* 或自定义运动原语的 State Lattice。TEB 可读其设计，但不宜作为新的 Humble 稳定依赖。

## 已确认事实

### Nav2 Smac

Nav2 在 2026 年仍维护 Humble、Jazzy、Kilted 和 Rolling，仓库持续发布，Smac 还有 2026 年的 cost-aware kinematically feasible planning 论文入口：[Nav2 仓库与引用](https://github.com/ros-navigation/navigation2)、[官方构建支持范围](https://docs.nav2.org/development_guides/build_docs/index.html)。

Smac Hybrid-A* 支持 Dubins/Reeds-Shepp、最小转弯半径、倒车/非直行惩罚、cost penalty 和 retrospective penalty；其现代性来自持续维护、代价感知启发式、SE(2) footprint 碰撞和工程优化，而不是 Hybrid-A* 名称本身。[官方配置](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)；[实现说明](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md)。

但 Humble 的 Smac Hybrid-A* 本质仍是“起点、终点、costmap”全局搜索，不天然接受一条参考路径并最小化 cross-track error。把它直接放进短视野局部重规划，空旷处可能选出另一条同样可行但偏离参考的路线。对四轮转向，Dubins/Reeds-Shepp 只表达等效曲率约束；若要表达 Ranger 的前后轮协同转向与可执行曲率集合，Smac State Lattice 的自定义 control set/motion primitives 更合适，但需实车参数确认。

### Nav2 MPPI critics

Humble `PathAlignCritic` 不是简单“最近点距离”：它按轨迹累计弧长匹配参考路径点，平均横向距离（可选航向差）；当尚未推进到 `offset_from_furthest` 时不启用；当参考段无效点比例超过 `max_path_occupancy_ratio` 且无效点多于 2 个时退出，让 obstacle/path-follow 接管。[Humble 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/path_align_critic.cpp)。

`PathFollowCritic` 负责进度，评分采样轨迹终点到“最远已达点之后的前方有效参考点”的距离；`PathAlign` 只负责贴线，两者不可互相替代。[当前源码](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/src/critics/path_follow_critic.cpp)。`CostCritic` 根据 costmap 累积远离障碍的代价，并可用完整 SE(2) footprint 检碰撞。[Humble CostCritic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/cost_critic.cpp)。

版本边界：新版本 `CostCritic` 的 `near_collision_cost` 临碰重罚不是 Humble 可直接配置的参数；Humble 只能借鉴该思想或自行实现。当前 Nav2 文档还支持插件式 motion model/trajectory validator，但这同样不能假定已回移到本项目 Humble 版本。[最新 MPPI 配置](https://docs.nav2.org/configuration/packages/configuring-mppic.html)。

### TEB

上游仍有社区活动，但默认分支是 ROS 1 Noetic；`ros2-master` README 仍声明面向 Dashing 和旧 Navigation2 commit，分支末端提交在 2024 年；`humble-devel` 末端提交在 2022 年。[ROS2 README](https://github.com/rst-tu-dortmund/teb_local_planner/blob/ros2-master/README.md)、[ros2-master](https://github.com/rst-tu-dortmund/teb_local_planner/tree/ros2-master)、[humble-devel](https://github.com/rst-tu-dortmund/teb_local_planner/tree/humble-devel)。因此没有足够依据把它当作 2025–2026 的 Humble 稳定首选。

## 对 Day5 的建议

障碍触发后的目标函数应显式拆成：

1. `progress`：持续向前，不因轻微 inflation 停滞；
2. `cross-track / PathAlign`：默认强贴参考；
3. `heading / PathAngle`：尤其约束弯道切角；
4. `collision / costmap`：footprint 碰撞为硬失败，膨胀代价为软代价；
5. `curvature + steering-change`：共享 Ranger 等效最小转弯半径并抑制突变；
6. `terminal rejoin`：在合理前向点平滑回接。

仅在参考路径局部占用率超过阈值的区段降低对齐权重，障碍后立即恢复；不要全局降低 `reference_deviation_weight`。RPP 可作为“强路径跟踪、无动态绕障”的对照基线，不承担局部避障。

现代走廊感知参考可看 Autoware MPT：它输入参考路径与左右 drivable-area bounds，在 Frenet/车辆模型中联合优化跟踪、边界、碰撞和转向稳定性；但模块重、计算较高且文档承认线性化与局部极小问题，只适合借鉴“显式走廊 + 失败后验证/停车”的设计，不建议直接移植。[Autoware Path Optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)、[Trajectory MPT](https://autowarefoundation.github.io/autoware_universe/latest/planning/autoware_trajectory_optimizer/)。

## 未验证项

- Ranger 前后轮转角关系、等效曲率和转向速率边界尚未统一进规划/控制模型。
- 本车 Humble 安装的 Nav2 精确版本与可用参数尚未核对。
- 最近 bag 已确认 72 次大偏离中 71 次参考段完全可通行，第一次发散由旧 `0.05 m` 位姿复用门槛触发；修复后的实车表现尚未验证。
- 候选算法的 CPU 周期、最坏规划时延和窄道成功率尚未在本车硬件测试。
