# Day 5 扫描—静态地图残差监测（2026-08-14）

## 1. 目的与边界

**事实**：`scan_map_residual_monitor_node` 只订阅 `/map`、`/scan`、`/odom`、`/Odometry` 和 TF，唯一主动发布的是字符串诊断主题 `/localization/scan_map_residual`。它不发布 TF、路径、速度或底盘控制命令。

**事实**：本工具不启用 AMCL，不修改 `map -> camera_init` 固定锚点，也不对 FAST-LIO 做在线纠偏。输出中的 `correction_*` 是“让当前扫描更贴合静态占用地图”的局部二维匹配估计，不是已经施加到定位链的变换。

## 2. 算法与输出

**事实**：监测器将静态 `OccupancyGrid` 转为欧氏距离场，再对当前扫描做粗到细的二维相关搜索。默认搜索范围为：

- 平移：粗搜索 `±0.40 m / 0.05 m`，细搜索 `±0.05 m / 0.01 m`；
- 航向：粗搜索 `±10° / 1°`，细搜索 `±1° / 0.25°`；
- 残差截断：`0.50 m`；
- 内点阈值：`0.10 m`；
- 最少有效扫描点：60。

每条 JSON 诊断至少包含：

- `correction_x_m`、`correction_y_m`、`correction_yaw_deg`；
- 原始与最佳匹配的均值/中位数/P90 残差；
- `inlier_ratio`、`point_count`、`search_boundary_hit`、`confident`；
- `scan_age_s`、`tf_age_s`、`vehicle_odom_age_s`、`fastlio_odom_age_s`；
- 静止观测时长、首尾平移/航向变化量与变化率；
- `classification`。

若最佳解落在搜索边界，`search_boundary_hit=true` 且 `confident=false`，不能把边界值当成真实偏差。

## 3. 静止分类语义

默认只有 `/odom` 新鲜度不超过 `0.5 s`、线速度不高于 `0.01 m/s` 且角速度不高于 `0.01 rad/s` 才积累静止样本。连续静止至少 300 秒后才形成稳定结论：

- `stationary_drift`：首尾校正量变化达到 `0.08 m` 或 `2°`；
- `fixed_anchor_offset`：校正量在窗口内稳定，但初始固定偏差达到 `0.08 m` 或 `2°`；
- `stationary_stable`：校正量小且稳定；
- `stationary_unstable`：变化不满足单调漂移阈值，但窗口波动过大；
- `insufficient_data`：连续静止时间不足；
- `low_confidence`：扫描—地图重合信息不足或最优解位于搜索边界；
- `moving_observation`：车辆速度反馈不满足静止条件，本条只记录、不纳入静止分类。

**建议**：`stationary_drift` 直接支持“FAST-LIO/时间同步在静止时仍漂移”的方向；`fixed_anchor_offset` 更支持初始锚定误差；`stationary_stable` 只能排除显著静止漂移，不能排除随运动累积漂移。

**未验证**：场内可动货架、人员、玻璃/稀疏结构和地图陈旧都可能降低置信度或造成局部极值。最终分类必须结合 RViz 重合情况、FAST-LIO 健康日志和现场物体是否移动解释。

## 4. 无运动运行方法

以下命令只启动残差监测节点；前提是静态地图、FAST-LIO、锚定 TF、`/scan` 和两个里程计主题已经存在。不要启动 terminal 8、relay 或底盘适配器。

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
cd /home/agilex/competition_ws
ros2 run competition_localization scan_map_residual_monitor_node \
  --ros-args \
  --params-file /home/agilex/competition_ws/config/localization/scan_map_residual_params.yaml
```

另一个终端只读确认节点接口：

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
ros2 node info /scan_map_residual_monitor
ros2 topic echo /localization/scan_map_residual
```

建议连续保存 5–10 分钟输出。若中途 `/odom` 判定为运动，静止窗口会清空并重新计时。

## 5. 当前验证状态

**事实**：合成占用地图测试已覆盖平移校正、航向校正、无效扫描过滤、搜索边界降置信度、静止漂移分类和固定锚定误差分类。

**未验证**：截至本文首次编写时，尚未在小车当前静止现场运行满 5 分钟，因此还不能据此宣布本次 FAST-LIO 偏差属于静止漂移、固定锚定误差或随运动累积漂移。

## 6. 2026-08-14 运动前后对照

### 6.1 采集条件

**事实**：用户重新发布初始位姿并明确允许发车后，由用户使用遥控器驾驶；本次没有启动自主路线 relay。车辆返回并停车后，先采集返回静止窗口，再停止 rosbag 和残差监测进程。

**事实**：原始记录保存在小车：

- rosbag：`/home/agilex/competition_ws/recordings/day5_motion_drift_20260814_200036`
- 监测日志：`/home/agilex/competition_ws/log/day5_motion_residual_throttled_20260814_195850.log`
- rosbag 时长 `203.790 s`，包含 `/tf`、`/tf_static`、`/Odometry`、`/odom`、`/scan` 和 `/localization/scan_map_residual`，共 20,469 条消息。

**事实**：现场有效扫描点约为 37–60，低于默认最少 60 点，因此本次只对监测节点临时覆盖 `min_points:=30`；未修改 AMCL、路线或定位 TF。

### 6.2 运动量与时间健康度

**事实**：按轮速 `|v| > 0.03 m/s` 或 `|w| > 0.05 rad/s` 划分，连续运动区间为 `78.893 s`。轮速积分路程为 `23.674 m`，FAST-LIO 以 5 Hz 抽样累计路程为 `23.751 m`；轮速积分累计绝对转角约 `412.9°`。两种路程估计相差约 `0.33%`。

**事实**：残差消息在运动前、运动中和返回后均持续输出。运动中 `scan_age_s`/`tf_age_s` 中位数为 `0.0385 s`、最大值为 `0.0551 s`，没有复现早先约 3.3 秒的监测器自身 TF 饥饿。该饥饿已由提交 `516d90c` 修复：失败的监测尝试也受 0.5 Hz 速率限制。

### 6.3 运动前后静态地图残差

以下只比较 `confident=true` 且未触碰搜索边界的静止帧，避免把 `±0.45 m` 搜索边界值当成真实校正量：

| 指标 | 运动前中位数（7 帧） | 返回后中位数（21 帧） | 返回后减运动前 |
| --- | ---: | ---: | ---: |
| `correction_x_m` | 0.32 m | 0.24 m | -0.08 m |
| `correction_y_m` | -0.11 m | -0.13 m | -0.02 m |
| `correction_yaw_deg` | 1.75° | 1.75° | 0.00° |
| `best_median_residual_m` | 0.0502 m | 0.0424 m | -0.0079 m |
| `inlier_ratio` | 0.650 | 0.673 | +0.023 |

**事实**：运动前可信帧的 `correction_x_m` 范围为 `0.09–0.44 m`，返回后为 `0.06–0.44 m`，两者高度重叠。返回后的全部 49 个静止帧中有 28 帧触碰搜索边界；运动前 10 帧中有 3 帧触碰边界。因此停车区对 x 方向的匹配可观性不足，不能使用全部帧的 x 中位数判断漂移。

**事实**：FAST-LIO 原始位姿的返回前后中位数相差约 `0.0346 m / 2.56°`。轮式里程计同期闭环差为 `3.52 m / -29.45°`，与人工返回现场不符，说明轮式里程计不能作为本次闭环真值；人工停车也没有毫米级复位基准。因此原始位姿闭环差只能作为辅助观察，不能单独归因于 FAST-LIO 漂移。

### 6.4 当前判断与下一步边界

**判断**：本次约 23.7 m、包含多次转向的运动没有检出运动后持续累积的 FAST-LIO 偏差。可信扫描—地图校正的航向中位数前后不变，平移变化落在单帧离散范围内，匹配残差没有恶化。当前更明显的问题是停车区扫描—地图匹配在 x 方向频繁触碰搜索边界，即局部可观性/地图重合度不足，而不是已经证实的 FAST-LIO 运动漂移。

**限制**：结论仅为“本次未检出”，不能证明更长路线、快速转弯或不同区域不会累积漂移；也尚未完成 300 秒连续静止分类。当前数据不足以把偏差正式定性为 `stationary_drift`、`fixed_anchor_offset` 或 `motion_accumulated_drift`。

**建议**：暂不启用 AMCL，也不继续调路线。下一次若需提高判别力，应在同一物理停车基准重复闭环，并扩大离线匹配搜索范围或选择二维结构更丰富的位置；任何再次运动仍需重新发布初始位姿并取得明确发车许可。
