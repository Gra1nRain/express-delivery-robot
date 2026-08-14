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
