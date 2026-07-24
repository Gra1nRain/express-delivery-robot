# Day5 局部规划长阻塞与转弯半径分析（2026-07-24）

## 基线与止损

- 已验证全程版本：`03d6b9a`。
- 固定标签：`day5-baseline-full-route-20260724`。
- 首次尝试的 `1.5 s` Python Hybrid A* 墙钟限制在车端误杀有效绕行，
  已由 `fece3f1` 回退；没有进入实车运动。

## 已确认事实

- 成功全程 bag：
  `recordings/day5_full_route_sensor_first_f54fa30_20260724_154700`。
- 最大 `local_replan_status` 间隙为 `70.363677 s`。
- 间隙前为 `REPLANNED`，间隙后为 `PLAN_FAILED`。
- 失败起终点为：
  `(5.323, 0.323, 0.041) -> (8.318, 0.372, 0.020)`。
- 自动提取的 `t=216.109786 s` costmap 能复现同一起终点。
- 现有 Python Hybrid A*、`20000` 扩展预算：
  - `0.81 m`：该帧无解；
  - `0.60 m`：该帧 `REPLANNED`，车端耗时约 `1.52 s`；
  - `0.50 m`：约 `1.35 s`；
  - `0.40 m`：约 `1.13 s`。
- Day5 控制配置记录的 Ranger 驱动最小转弯半径为 `0.47644 m`。
- 车端官方 `ranger_nav` 参数将底盘描述为 skid-steer；当前 Day5 安全策略
  仍明确要求 dual Ackermann 模式，因此本轮不启用原地转或倒车。

## Nav2 Smac 原型结论

throwaway 分支：`prototype/day5-smac-benchmark`，结论提交 `ee01a3c`。

在修正项目栅格“顶部首行”和 Nav2 costmap“底部首行”的转换后：

- DUBIN、`0.81 m` 会拒绝已知恢复帧，不能直接替换现规划器。
- REEDS_SHEPP 能通过三帧，但需要约 `0.066 m` 和 `0.011 m` 倒车；
  当前轨迹参数化与 MPPI 只支持非负速度，不能直接接入。
- DUBIN、`0.60/0.50/0.40 m` 均能在三帧上无倒车成功，搜索低于 `3 ms`。

参考实现：
[Nav2 Smac Hybrid-A* 官方配置](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)。

## 本轮决定

- 冻结全局路线规划继续使用 `0.81 m`，不改变轨迹 artifact 与来源指纹。
- 在线局部规划、MPPI 和 safety 统一使用 `0.60 m`。
- `0.60 m` 高于驱动记录的 `0.47644 m`，且已有 Ranger Twist 适配测试覆盖。
- 不加入固定墙钟超时，不启用倒车，不更换在线规划器。

## 未验证

- `0.60 m` 局部路径尚未进行新一轮全段实车验证。
- 实车验收重点是全段完成、索引 56 附近不再出现几十秒状态间隙，以及峰值
  横向/航向误差不劣于基线。
- 若出现更长阻塞、控制振荡、驱动饱和或整体误差恶化，应停止并回到
  `day5-baseline-full-route-20260724`，不继续叠加调参。
