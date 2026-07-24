# Day5 局部轨迹无障碍偏离根因（2026-07-24）

## 结论

局部轨迹在转弯处偏离全局轨迹的主要原因不是 local costmap 阻断，而是局部规划器的模式切换：

1. 全局参考段可通行。
2. 车辆相对参考点的位置误差刚超过 `0.05 m`，局部规划器不再复用参考段。
3. 规划器改为从当前姿态自由搜索到约 `3 m` 外的单个 rejoin 点。
4. 第一次自由搜索把约 `0.063 m` 的跟踪偏差放大为 `0.223 m` 的局部路径偏差。
5. 车辆跟随偏离后的局部路径，后续重规划从更大的姿态误差继续搜索，形成正反馈并最终放大到约 `1.36 m`。

修复原则不是把局部轨迹硬夹在固定走廊内，而是：参考段可通行时，参考段本身就是无障碍条件下参考距离代价为零的最优候选；只有参考段实际被阻断时才进入绕行搜索。

## 数据来源

- Bag：
  `/home/agilex/competition_ws/recordings/day5_remaining_route_adapter_905e409_20260724_112219`
- 离线分析入口：
  `scripts/analyze_day5_local_replanning_bag.py`
- 分析方式：用 `rosbag2_py` 直接读取 bag，不播放任何话题。
- 规划器膨胀半径：`0.45 m`
- 偏离诊断阈值：`0.50 m`

## 已确认事实

离线反馈环输出：

```text
status=FAIL
events=161
high_deviation=72
high_clear=71
high_blocked=1
peak_m=1.363449115054148
peak_t_s=116.619
reference=116:147
blocked_reference=0/32
```

解释：

- 161 次 `REPLANNED` 中，72 次局部轨迹到全局轨迹的最大距离超过 `0.50 m`。
- 72 次大偏离中，71 次的整段全局参考在静态图和当时 local costmap 上均未被阻断。
- 峰值时参考索引为 `116 -> 147`，32 个参考点全部可通行。
- Bag 中 TF 边实际为：
  - `map -> camera_init`
  - `camera_init -> body`
  - `odom -> base_link`

## 第一次发散

`t=30.898870344 s`：

- 参考索引：`71 -> 102`
- 当前位姿到参考起点距离：`0.0631479 m`
- 当前航向到参考航向误差：`0.0725 deg`
- 参考段被阻断点数：`0`
- 规划器输出最大参考偏离：`0.2233018 m`

该位置误差刚超过旧实现的 `reuse_position_tolerance_m=0.05`，触发从 `REFERENCE_CLEAR` 到 `REPLANNED` 的不连续切换。

随后：

- `t=78.5586 s`：无动态障碍，偏离已增至 `0.5589 m`。
- `t=116.6191 s`：偏离达到 `1.3634 m`，参考段仍完全可通行。

## 确定性最小复现

在 PC 上使用真实静态地图、真实 193 点轨迹和 `t=30.8989 s` 当前位姿，不叠加任何动态障碍：

```text
status=REPLANNED
reference_start_index=71
rejoin_index=102
max_local_to_global_deviation_m=0.22330181774743593
```

该复现不依赖 ROS、TF、点云或实车，运行时间小于 1 秒。它已固化为
`test_clear_day5_turn_reference_is_kept_after_small_tracking_offset`。

## 单变量验证

对 `t=78.5586 s` 的无障碍输入：

- 当前实现稳定复现约 `0.577 m` 最大偏离。
- `reference_deviation_weight` 从 `0` 增加到 `50`，输出路径不变。
- `lookahead` 为 `1.0–2.0 m` 时无解，`2.5–4.0 m` 时偏离仍约 `0.577 m`。
- 最小转弯半径越大，偏离越大。
- `curvature_bins` 越多，当前“一步只变化一个 bin”的隐式曲率变化约束越严格，偏离反而越大。

因此：

- “参考权重太小”不是主要根因。
- 当前 Hybrid A* 的单 rejoin 目标和离散曲率转换约束不适合承担无障碍跟踪误差恢复。
- 这些问题仍影响真正绕障时的路径质量，但不应在无障碍时被触发。

## 已实施修复

`LocalTrajectoryPlanner.plan()` 现在：

1. 叠加静态地图和实时障碍。
2. 检查选定全局参考段在同一膨胀语义下是否可通行。
3. 可通行时直接返回 `REFERENCE_CLEAR` 和原参考段。
4. 仅在参考段被阻断时调用 Hybrid A* 绕行并回接。

该改变保留了局部绕障能力，没有添加最大偏离硬限制。

## 验证

```text
python -m pytest \
  tests/test_local_trajectory_planner.py \
  tests/test_day5_local_replanning_bag_analysis.py \
  tests/test_day5_online_replanning_topology.py -q

15 passed
```

## 风险与后续

- 历史 bag 记录的是修复前输出，离线偏离断言仍应返回 FAIL；它是回归证据，不是修复后运行结果。
- 当参考段被真实障碍阻断时，现有 Hybrid A* 仍需要更现代的参考走廊/车辆模型代价设计。
- 如果定位跳变导致车辆离全局参考很远，直接使用参考段可能触发 MPPI 或 safety 的跟踪误差保护；这比局部规划器静默重定义参考更可观测，但应在无运动回放和后续受控试验中验证。
- 尚未进行修复后的实车运动验证。
