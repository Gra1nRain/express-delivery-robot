# Day 5 室内约双倍巡航速度试验方案（2026-08-17）

## 事实

- 基线轨迹：`debug_indoor_one_lap_continuous_trajectory_8_14_1.yaml`。
- 提速轨迹：`debug_indoor_one_lap_continuous_trajectory_8_17_1.yaml`。
- 两份轨迹的 236 个 `x/y/yaw/s/curvature/ref_id` 完全相同；本次不改变路线几何、固定初始位姿或停车点。
- 基线最高参考速度为 `0.1093 m/s`，提速轨迹最高参考速度为 `0.2000 m/s`，提高 `82.98%`。
- 基线平均参考速度为 `0.1088 m/s`，提速轨迹平均参考速度为 `0.1925 m/s`，提高 `76.94%`。
- 离线计划时长由 `216.959 s` 降至 `122.620 s`，减少 `43.48%`。
- 提速轨迹有 214/236 个采样点达到 `0.20 m/s`；曲率突变附近最低保持 `0.1093 m/s`。
- 提速轨迹最大参考加速度为 `0.160 m/s²`，最大参考 jerk 为 `0.365 m/s³`，最大曲率变化率为 `0.800 1/m/s`。
- `config/safety/safety_params.yaml` 不修改；急停、安全监督、规划失败停车、动态障碍检查和 `0.20 m/s` 安全硬上限全部保留。
- 精停速度和检查点控制逻辑不修改。

## 确定性闭环仿真对比

| 指标 | `8_14_1` 基线 | `8_17_1` 提速 | 结果 |
|---|---:|---:|---|
| 仿真整圈时间 | `235.75 s` | `130.95 s` | 减少 `44.45%` |
| 峰值车速 | `0.1365 m/s` | `0.1976 m/s` | 提高 `44.73%` |
| 最大横向误差 | `0.0185 m` | `0.0222 m` | 增加 `0.0036 m` |
| 最大航向误差 | `4.65°` | `5.18°` | 增加 `0.53°` |
| 最终位置误差 | `0.0865 m` | `0.0776 m` | 改善 `0.0090 m` |
| SAFE_HOLD 次数 | `0` | `0` | 无退化 |

以上仅为同一确定性运动学模型内的相对比较；实车稳定性结论仍需首圈日志确认。

## 启动参数

主控制栈在原连续一圈启动命令中替换或增加以下参数：

```bash
trajectory_file:=/home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory_8_17_1.yaml \
optimizer_params_file:=/home/agilex/competition_ws/config/planning/optimizer_params_day5_speed_2x.yaml \
control_params_file:=/home/agilex/competition_ws/config/control/control_params_day5_speed_2x.yaml \
safety_params_file:=/home/agilex/competition_ws/config/safety/safety_params.yaml
```

状态机中继的 `--route-file` 必须同步指向：

```bash
/home/agilex/competition_ws/docs/evidence/day5/debug_indoor_one_lap_continuous_trajectory_8_17_1.yaml
```

回退时同时切回 `debug_indoor_one_lap_continuous_trajectory_8_14_1.yaml`、`optimizer_params.yaml` 和 `control_params.yaml`；不要混用提速轨迹与基线 `0.15 m/s` 控制上限。

## 首圈验收与立即回退门槛

- 发车前必须重新发布固定初始位姿，并由用户明确说“可以发车”。
- 只运行一圈；失败后先保存中继和诊断日志，不连续重试。
- 任意 `LOCAL_PLAN_STALE`、`LOCAL_PLANNER_STOP`、持续 `SAFE_HOLD`、proximity stop 或动态障碍停车均保留证据并结束本圈。
- 横向误差达到 `0.30 m`、航向误差达到 `30°`、安全命令超过 `0.23 m/s` 或里程计速度超过 `0.28 m/s` 时立即停车回退。
- 任一精停点越线、停车姿态不满足现有门槛或最终停车精度退化时回退。
- 通过后比较整圈耗时、移动段平均/峰值里程计速度、误差均值/P95/峰值、零速事件数量与持续时间、局部规划超时/过期次数以及最小障碍净空。

## 未验证

- `122.620 s` 是离线参考时长，不代表实车整圈时间。
- 实车平均速度提升、短暂停顿是否减少、跟踪误差和最小净空是否保持，必须由首圈日志确认。
