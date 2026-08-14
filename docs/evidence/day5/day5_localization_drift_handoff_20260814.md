# Day 5 定位漂移与货架贴碰问题交接（2026-08-14）

## 1. 新对话的首要目标

先暂停自主发车，完成以下两项工作：

1. 定量确认实时雷达扫描相对静态地图的误差，是固定锚定误差、静止漂移，还是随运动累积的 FAST-LIO 漂移。
2. 修复精停固定参考会绕过局部规划硬停止的问题。在这项修复完成并通过无运动验证前，不再进行贴货架自主测试。

不要先继续调整全局路线、rear 延长距离或货架膨胀参数。这些都不是最后一次贴架事件的直接触发点。

## 2. 当前代码与小车状态（事实）

- 电脑源码权威目录：`E:\Myself\Project\无人系统大赛\competition_car`
- 当前分支：`feature/piper-arm-integration`
- 当前功能基线提交：`15b9bc0 fix(control): clear shelf inflation after rear docks`（交接文档本身会形成后续独立提交）
- 分支相对远端：ahead 40
- 小车工作区：`/home/agilex/competition_ws`
- `15b9bc0` 的运行文件和两份测试已定向同步到小车；10 个同步文件 SHA-256 一致。
- 小车端构建通过：`competition_planning`、`competition_control`、`competition_bringup`。
- 电脑端测试：`python -m pytest tests -q`，`270 passed`。
- 小车端针对性测试：`77 passed`。
- 交接时已确认没有 `day5_full_route_relay.py`、`day5_motion_control.launch.py`、`mppi_control` 或 `local_replanner` 运动进程在运行。
- 用户已经人工停车并接管车辆。

以下未跟踪内容属于用户或既有现场证据，不要删除、覆盖或纳入本任务提交：

- `Piper_Grasp_Humble_Migration_20260723/`
- `docs/evidence/day5/day5_sensor_visual_baseline_20260812.md`
- `docs/evidence/day5/runs/`
- `scripts/.codex_analyze_run.py`

## 3. 当前已实现的精停与路线行为（事实）

相关提交按时间倒序：

- `15b9bc0`：rear 停靠点附近继续使用货架专用碰撞几何，离开货架时不再被对应货架膨胀卡住。
- `ec6b1d9`：front 完成货架相对精停后，rear 从实际 front 停车位沿标定航向直行，不重新拟合货架。
- `f99d6a6`：拒绝远处背景被误识别为货架。
- `efd7b7d`：精停阶段融合稀疏扫描。
- `cce3f43`：货架相对精停基础实现。

用户明确要求并已写入配置：

- 不增加新的离场运动状态。
- 不增加多帧放行门禁。
- `pickup_front -> pickup_rear` 与 `drop_front -> drop_rear` 的直行距离都比语义点间距增加 `0.11 m`。
- 当前理论距离约为：pickup `0.710 m`，drop `0.691 m`。
- rear 完成后只忽略对应货架膨胀；其他动态障碍和底盘安全保护应保留。

配置位置：

- `config/docking/debug_dock_params.yaml`
- `config/docking/competition_dock_params.yaml`
- `config/planning/local_hybrid_astar_runtime_params_day5.yaml`

最后一次事故发生在到达 `pickup_front` 之前，因此 `0.11 m` rear 延长和 rear 离场过滤尚未激活，不能把事故归因于这两项新改动。

## 4. 最后一次实车运行证据（事实）

运行标签：`state_machine_one_lap_20260814_190331`

小车端证据文件：

- `/home/agilex/competition_ws/log/state_machine_one_lap_20260814_190331.jsonl`（约 126 MB）
- `/home/agilex/competition_ws/log/state_machine_one_lap_20260814_190331_status.json`
- `/home/agilex/competition_ws/log/state_machine_one_lap_20260814_190331_final_status.json`
- `/home/agilex/competition_ws/log/state_machine_one_lap_20260814_190331_summary.txt`

用户现场观察：

- 实车已经紧贴货架。
- 用户最初描述“地图中位置正常”，随后更正为：实时雷达红色扫描线与静态地图并不完全重合。
- 用户人工把车停下。

日志关键时间线：

- 约 `81 s`：普通局部规划短暂进入 `SAFETY_HOLD`，随后恢复。
- 约 `86 s`：进入 `pickup_front / PRECISION_APPROACH`。
- 约 `93.43 s`：局部规划持续无可行路径后，安全输出速度降为 0。
- `93.43–147.46 s`：车辆基本保持不动。
- 约 `147.46 s`：精停固定参考再次开始给出前进速度，速度从约 `0.01 m/s` 增至约 `0.10 m/s`。
- 用户随后人工停车。

最终状态中的关键字段：

```text
active_checkpoint_ref = pickup_front
precision_phase = PRECISION_APPROACH
control_status = TRACKING
map executed pose ~= (7.556, -0.028)
pickup_front semantic pose = (8.413, -0.081)
local planner = HYBRID_ASTAR_NO_FEASIBLE_PATH
local detail = start pose (7.554, -0.030) is blocked
docking_mode = true
local stop_requested = true
docking_filtered_obstacle_count = 3
proximity_stop = false
proximity reason = clear
scan_min ~= 0.346 m
```

这证明局部规划已经认为当前车体位姿被占用并请求停止，但精停控制仍报告 `TRACKING`。

## 5. 已确认的控制安全缺陷（事实）

文件：`src/competition_control/competition_control/mppi_control_node.py`

当前逻辑在 `_segmented_command()` 中计算：

```python
local_plan_unavailable = (
    self._replanning_enabled
    and not self._precision_active
    and (... local stop / local plan stale ...)
)
```

当 `self._precision_active` 为 true 时，局部规划停止请求不会进入 `local_plan_unavailable`。之后精停固定参考分支会直接返回跟踪命令。因此出现了：

- 局部规划：`start pose is blocked`、`stop_requested=true`
- 精停控制：仍可在等待后重新输出前进命令

这不是理论推测，而是最后一次日志已经复现的实际行为。

用户之前要求“精停路段忽略货架膨胀”，正确语义应为：只在规划层过滤预期货架膨胀，不应绕过“当前车体已经碰撞/起点被占用”、近障停车或其他真实障碍的硬停止。

建议下一对话先基于这段真实时间线建立回归测试，再修改控制逻辑。修复应至少保证：

- `precision_active=true` 且局部规划报告当前起点被占用时，输出必须保持为零。
- 预期货架回波过滤仍可工作。
- `/avoidance/proximity_stop` 仍拥有最高停止优先级。
- 不要把所有 `HYBRID_ASTAR_NO_FEASIBLE_PATH` 不加区分地永久锁死；需要区分当前车体碰撞与远端目标/搜索失败。

## 6. 当前定位链为何不会自动贴回静态地图（事实）

当前 Day 5 权威定位链：

```text
map -> camera_init -> body
```

- `camera_init -> body`：FAST-LIO 使用雷达和 IMU 连续估计。
- `map -> camera_init`：`competition_localization/fastlio_anchor_node.py` 在收到一次 `/initialpose` 后计算。
- `fastlio_anchor_node` 计算出 `_map_to_odom` 后，只按固定频率重复发布同一个变换；不会继续将实时 `/scan` 与静态栅格地图匹配。
- Day 5 不启动 AMCL。
- RANGER `/odom` 只作为速度反馈，不发布竞争性的全局定位 TF，也不负责把 FAST-LIO 拉回静态地图。

因此 FAST-LIO 虽然有雷达—IMU局部匹配和内部误差修正，但系统没有“相对赛场静态地图”的全局闭环。若 `camera_init -> body` 累积漂移，红色扫描线会逐渐偏离静态地图，固定的 `map -> camera_init` 不会自动改变。

相关依据：

- `src/competition_localization/competition_localization/fastlio_anchor_node.py`
- `docs/interface-contract.md`
- `docs/day2.md` 中关于 AMCL 不稳定和改用 FAST-LIO 固定锚定的记录

## 7. 定位偏差的待验证假设（不是结论）

按优先级建议检查：

1. **FAST-LIO 随运动累积漂移，且系统没有静态地图闭环。**
   - 预测：起点对齐后误差较小；遥控或自主行驶距离增加时，扫描—地图残差持续增大；重新发布 `/initialpose` 会整体恢复。
2. **传感器时间同步、队列积压或计算调度再次异常。**
   - 预测：偏差增长同时伴随点云/IMU时间年龄、频率或 `FAST_LIO_HEALTH` 指标异常。
   - 已有积压修复证据：`docs/evidence/day5/day5_fastlio_backlog_remediation_20260812.md`，但不能假定所有运行状态下都永久正常。
3. **初始锚定本身存在位置或航向小误差。**
   - 预测：误差从发车前就存在，随后大致保持固定，而不是随里程增长。
4. **静态地图与现场货架实际位置发生变化。**
   - 预测：只在某些物体附近出现固定偏差，其他墙面/固定结构仍重合。
5. **`body`、雷达外参或 footprint 尺寸不准确。**
   - 这与扫描—地图漂移是两个可以同时存在的问题。即使红线完全重合，错误的车体外廓仍可能造成贴架。

不要通过“关机休息后看起来恢复”来判定问题解决。重启 FAST-LIO 或重发 `/initialpose` 只会重置状态/锚点，无法证明漂移根因消失。

## 8. 下一对话建议的无运动诊断闭环

第一阶段不要启动终端8/relay，不向底盘发布运动命令。

1. 按已验证顺序启动 Livox、FAST-LIO、锚定/地图/RViz，但不启动运动控制输出。
2. 用户在可靠位置发布 `/initialpose`，记录发车前扫描—地图是否重合。
3. 同时采集：
   - `/tf`、`/tf_static`
   - FAST-LIO `/Odometry`
   - `/cloud_registered_body`
   - `/scan`
   - 实际 IMU 主题（先用 `ros2 topic list` 确认名称）
   - `FAST_LIO_HEALTH` 日志或等价运行健康指标
4. 建立一个可重复的“扫描—静态地图残差”指标，至少输出平移误差、航向误差和时间戳年龄。不要只靠肉眼看 RViz。
5. 先静止观察 5–10 分钟：
   - 静止也持续漂移：优先查 IMU偏置、时间同步、队列和 FAST-LIO 状态。
   - 静止稳定、运动后增长：优先查运动退化场景、慢速微动、振动、点云几何约束和无全局闭环。
6. 由用户遥控完成一段低风险路线并返回已知位置；Codex 只监测，不主动发车。
7. 比较起点、途中、返回后的扫描—地图残差，再决定采用哪种全局纠偏方案。

候选全局纠偏方案需要数据后再选：

- 重新整定并启用 AMCL（必须避免与 FAST-LIO anchor 同时发布冲突 TF）。
- 使用 `slam_toolbox` localization mode 或其他 2D scan-to-map matcher。
- 对 FAST-LIO 点云/PCD 做低频 ICP/NDT 全局匹配。
- 在可靠语义精停点停车后进行受门控的局部地图校正。

推荐约束：全局校正应低频、带置信度阈值、限制单次平移/旋转跳变，并优先在停车或低速时应用，避免控制过程中突然跳 TF。

## 9. 实车安全与操作约束

- 不使用 `scripts/restart.sh`，该脚本已被用户明确判定有问题。
- 节点启动顺序可能影响 FAST-LIO 数据新鲜度，应继续参考 `/home/agilex/agilex_ws/test.md` 和当前手动测试文档。
- 电脑端是源码与 Git 权威副本；修改后先测试、提交，再通过既有 `scp` 方案定向同步到 `/home/agilex/competition_ws`，不要使用压缩包。
- 同步排除 `.git/`、`build/`、`install/`、`log/`、`recordings/` 和缓存。
- 无运动检查、构建和测试可以直接做。
- 任何可能让底盘运动的操作，必须先获得用户明确允许，并由用户重新发布、确认初始位姿。
- 用户暂停或接管时，立即停止 relay，不得等待规划器自行恢复。
- 失败后先保留日志，不连续盲目重试。
- 下一次贴货架测试前，至少先完成精停硬停止修复和无运动验证。

## 10. 新对话可直接使用的开场提示

```text
请先完整阅读：
E:\Myself\Project\无人系统大赛\competition_car\docs\evidence\day5\day5_localization_drift_handoff_20260814.md

继续处理 Day 5 实车定位漂移与精停安全问题。先只读检查，不发车：
1. 用最后一次日志建立“精停固定参考绕过局部规划 start pose blocked”的回归测试并修复；
2. 设计并实现无运动的扫描—静态地图残差监测，确认 FAST-LIO 漂移类型；
3. 在数据充分前不要直接开启 AMCL或继续调路线；
4. 任何实车运动必须等我重新发布初始位姿并明确说可以发车。

最后一次日志：
/home/agilex/competition_ws/log/state_machine_one_lap_20260814_190331.jsonl
当前功能基线提交：15b9bc0
```
