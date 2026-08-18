# 室内比赛完整流程运行与恢复手册

## 1. 本文档对应的基线

本文档固化 2026-08-19 的室内比赛慢速全流程基线。操作员已在真实小车上确认以下闭环可以连续运行：

```text
挥旗启动
→ 前往红绿灯并按灯态启停
→ 接近前/后抓取点
→ 识别指令图片和目标货物
→ 抓取并保持夹紧
→ 接近前/后放置点
→ 识别放置目标并释放货物
→ 前往终点并停车
```

- 电脑端源码权威副本：`E:\Myself\Project\无人系统大赛\competition_car`
- 小车端运行目录：`/home/agilex/competition_ws`
- Git 分支：`feature/piper-arm-integration`
- 行为代码基线：`a0cbb8d fix(mission): overlap pickup vision preload with driving`
- 固化标签：`verified/full-flow-slow-20260819`
- 验证结论边界：这是操作员现场确认的单套场地、当前摆放和慢速参数下的完整闭环，不代表正式赛场、所有目标类别或所有失败恢复分支都已实车覆盖。

> 当前分支在该标签之后把巡航参考上限调整为 `1.0 m/s`，目标点前 `1.0 m` 降至现有 `0.08–0.12 m/s` 精确进场。该高速配置已完成离线轨迹和自动测试，但尚未实车验证；标签中的约 `0.119 m/s` 慢速版本保持不变，可随时恢复。

本文档不替代更细的开发文档。状态机设计见 `docs/competition_mission_state_machine.md`，历史手动测试步骤见 `docs/competition_mission_manual_field_test.md`。

## 2. 系统组成和资源约束

| 模块 | 入口或资源 | 说明 |
|---|---|---|
| 整车启动 | `competition_bringup/indoor_competition.launch.py` | 状态机、规划、控制、安全、感知和机械臂统一入口 |
| Ranger 底盘 | `can3`，500 kbit/s | 必须有持续 CAN 回包，否则不允许发车 |
| Piper 机械臂 | `can2`，1 Mbit/s | 启动后自动到统一行驶/观察位姿 |
| 腕部相机 | `/left_wrist_camera/...` | 小车红绿灯与机械臂共用一个 RealSense，不启动第二个相机 |
| 红绿灯感知 | `/perception/wrist_traffic_annotated` | 挥旗和红绿灯标注画面 |
| 机械臂感知 | `/perception/arm_recognition_annotated` | 图片/物体/放置识别、阶段和结果标注 |
| 任务状态 | `/mission/status` | 主状态、货物状态、ArmTask 阶段和失败原因 |
| 机械臂任务 | `/mission/arm_task` | 常驻 `ArmTask` Action，服务端只能有一个 |

YOLO 只在红绿灯放行后开始用 GPU `device=0` 预加载，并与前往抓取点同步进行；抓取开始前仍会等待模型就绪。GroundingDINO、SAM 等非 YOLO 推理保持使用 CPU。相机由整车 launch 常驻管理，机械臂适配器以 `manage_camera=false` 复用现有话题。

## 3. 当前关键参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| 腕部 RGB 画面 | `640×480 @ 15 Hz` | 标注画面也以最高 15 Hz 更新，实际推理频率受算力限制 |
| 红绿灯预热点 | 停车点前 `1.0 m` | 只启动识别，不停车；真实停车点才停 |
| 红绿灯无结果放行 | `15.0 s` | `UNKNOWN/OFF` 连续超时后继续比赛 |
| 抓取 Action | 每停车点 `1` 次，合计超时 `180 s` | 前点失败才进入后点 |
| 放置 Action | 每停车点 `1` 次，合计超时 `90 s` | 前点失败必须再观察后点 |
| 比赛参考速度上限 | `1.00 m/s` | 直线尽量达到上限，曲线按横向加速度包络降速 |
| 控制器与 Safety 上限 | `1.00 m/s` | 两层一致，底盘协议配置上限仍为 `2.00 m/s` |
| 加速/减速上限 | `0.50/0.80 m/s²` | 从 `1.00` 降至 `0.12 m/s` 的理论制动距离约 `0.616 m` |
| 速度 jerk 上限 | `2.00 m/s³` | MPPI 指令按 S 曲线改变加速度，避免加减速阶跃 |
| 目标点减速区 | `1.00 m` | 区内保持现有 `0.08–0.12 m/s` 进场速度 |
| 方块额外下探 | `0.020 m` | 只应用于方块，瓶子策略不变 |
| 瓶身抓取高度比例 | `0.25` | 位于瓶身下半部 |
| 瓶底最小余量 | `0.020 m` | 所有补偿后的最终抓取中心约束 |
| 瓶子前探补偿 | `0.055 m` | 保留直立瓶侧向抓取策略 |

当前离线轨迹保持原有 236 点几何不变，最高速度 `1.000 m/s`、最大绝对 jerk `1.982 m/s³`，除起点和 `finish_park` 外没有零速点；普通弯道继续保留正向速度。该平滑参数和新轨迹尚待实车复验。

统一行驶/红绿灯观察位姿为：

```text
joints_rad: [0.005760, 0.289742, -0.565347, -0.081856, 0.045605, 0.092502]
joints_deg: [0.330, 16.601, -32.392, -4.690, 2.613, 5.300]
```

机械臂启动和任务结束回到此位姿时保持当前夹爪开度。抓取成功后保持 `0.0000 m` 闭合目标及夹紧力，运输途中不松爪；只有明确进入放置释放步骤才允许张开。

## 4. 终端规划

推荐使用 5 个终端：

| 终端 | 用途 |
|---|---|
| A | CAN 检查和整车 launch 日志 |
| B | 发布初始位姿、发车前检查和必要时手动触发 |
| C | 持续查看 `/mission/status` |
| D | `rqt_gui`，集中显示三路相机画面 |
| E | RViz 和运行期只读诊断 |

每个新终端都先执行：

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh
export CAN_NAME=can2
export PIPER_CAN_NAME=can2
```

必须退出 Conda。ROS Humble 使用 Python 3.10；在 Conda Python 3.14 中运行 `rqt_image_view` 会出现 `rclpy._rclpy_pybind11` 无法导入。

## 5. 启动前清理和 CAN 准备

只检查并结束上一轮的准确 PID，不使用宽泛的 `pkill`：

```bash
pgrep -af 'indoor_competition|mission_node|piper_arm_task_node|piper_single_ctrl|arm_task_simulator_node|ranger_twist_adapter|ranger_base_node' || true
```

若确认某个 PID 属于上一轮，再执行：

```bash
kill -INT <准确的PID>
```

配置两个 CAN 口：

```bash
sudo ip link set can3 down 2>/dev/null || true
sudo ip link set can3 type can bitrate 500000 restart-ms 100
sudo ip link set can3 txqueuelen 1000
sudo ip link set can3 up

sudo ip link set can2 down 2>/dev/null || true
sudo ip link set can2 type can bitrate 1000000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up

ip -details -statistics link show can3
ip -details -statistics link show can2
timeout 3s candump -L can3
```

`can3` 必须收到 Ranger 回包。若 `candump` 完全空白、`/system_state` 显示 `control_mode: 0`、`battery_voltage: 0.0`，不要发车；优先检查急停、遥控器、底盘供电、CAN 接口对应关系和 USB-CAN 连接。

## 6. 启动整车

终端 A：

```bash
mkdir -p /home/agilex/competition_ws/log
ros2 launch competition_bringup indoor_competition.launch.py \
  start_base:=true \
  start_chassis_adapter:=true \
  start_wrist_camera:=true \
  start_real_arm:=true \
  start_arm_simulator:=false \
  arm_post_instruction_clear_delay_s:=10.0 \
  rviz:=false \
  2>&1 | tee "/home/agilex/competition_ws/log/competition_manual_$(date +%Y%m%d_%H%M%S).log"
```

`arm_post_instruction_clear_delay_s:=10.0` 用于室内人工展示图片：识别并锁定目标类型后留出 10 秒移走图片。正式自动比赛无需人工移图时可改回 `0.0`。

整车 launch 会连接真底盘和真机械臂。启动后机械臂会自动移动到统一位姿，但状态机仍应停在 `WAIT_START_FLAG`，挥旗前 `/cmd_vel` 必须为零。

若 RealSense 报 `Device or resource busy`，说明还有其他进程占用相机。保留日志，结束本轮 launch，查出准确占用进程后再重新启动；不要同时运行第二个 RealSense launch。`rqt_gui` 只是订阅图像，不会占用设备。

## 7. 发布初始位姿并检查地图对齐

把车放回已标记的相同物理起点和方向。终端 B：

```bash
ros2 topic pub --once \
  --wait-matching-subscriptions 3 \
  --keep-alive 2.0 \
  /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -0.416, y: 0.464, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0003046]}}"
```

此坐标只适用于当前已标记物理起点，不是轨迹 YAML 首点。发布后等待至少 1 秒，在 RViz 中确认雷达扫描与静态地图重合；不重合时不要挥旗。

终端 E 打开 RViz：

```bash
rviz2 -d /home/agilex/competition_ws/install/competition_bringup/share/competition_bringup/rviz/day5_motion_control.rviz
```

若 RViz 提示消息时间早于 TF cache，先确认系统时间、`map -> body` TF 和定位节点都在持续更新；不要反复发布不同初始位姿掩盖问题。

## 8. 发车前只读检查

终端 B 依次执行：

```bash
ros2 topic echo /mission/status --once --qos-durability transient_local --full-length
ros2 action info /mission/arm_task
ros2 topic echo /control/state_valid --once
ros2 topic echo /avoidance/stop_request --once
ros2 topic echo /system_state --once
ros2 topic echo /safety/event --once
ros2 topic echo /cmd_vel --once
timeout 3s ros2 run tf2_ros tf2_echo map body
timeout 5s ros2 topic hz /left_wrist_camera/camera/color/image_raw
```

允许挥旗前应同时满足：

- `/mission/status` 为 `WAIT_START_FLAG`。
- `/mission/arm_task` 只有一个 Action server，节点为 `/piper_arm_task`。
- `/control/state_valid` 为 `true`。
- `/avoidance/stop_request` 为 `false`。
- `/system_state` 的 `control_mode: 1`、电压非零、`error_code: 0`、`motion_mode: 0`。
- `/safety/event` 没有 `remote_not_ready`、`stale_command` 或 `stale_state` 等原因。
- 挥旗前 `/cmd_vel` 为零。
- `map -> body` 可查询，腕部相机持续出图。

任一项不满足时不要发车，保留终端 A 的日志并诊断。

## 9. 图像和状态集中可视化

终端 D 启动一个 `rqt_gui`：

```bash
ros2 run rqt_gui rqt_gui
```

在同一窗口中选择三次 `Plugins → Visualization → Image View`，分别订阅：

```text
/left_wrist_camera/camera/color/image_raw
/perception/wrist_traffic_annotated
/perception/arm_recognition_annotated
```

使用 `Perspectives → Export` 保存布局，下次可以直接恢复同一个界面。若下拉框能选中话题但画面灰白，检查：

```bash
ros2 topic info /left_wrist_camera/camera/color/image_raw -v
timeout 5s ros2 topic hz /left_wrist_camera/camera/color/image_raw
ros2 topic info /perception/wrist_traffic_annotated -v
ros2 topic info /perception/arm_recognition_annotated -v
```

机械臂标注图至少显示任务、阶段、目标类型、attempt、置信度、bbox、抓取中心和最终结果。`INSTRUCTION IMAGE RECOGNITION` 是纸片识别阶段，`TARGET OBJECT RECOGNITION` 是实物识别/定位阶段。

终端 C 持续监视状态：

```bash
ros2 topic echo /mission/status --full-length
```

## 10. 启动比赛和完整状态流

优先使用真实挥旗。识别成功后画面应显示 `FLAG: DONE`、`START: CONFIRMED`，任务状态离开 `WAIT_START_FLAG`。

仅在已经明确允许实车运动且需要绕过视觉验证时，才使用等价的手动事件：

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /perception/flag_wave_detected std_msgs/msg/Bool \
  "{data: true}"
```

前点装卸成功时的主状态链：

```text
WAIT_START_FLAG
→ RUN_TO_TRAFFIC_STOP
→ WAIT_TRAFFIC_LIGHT
→ RUN_TO_PICKUP_FRONT
→ PICKUP_FRONT_TASK
→ RUN_TO_DROP_FRONT
→ DROP_FRONT_TASK
→ RUN_TO_FINISH
→ FINISHED
```

分支规则：

- 红绿灯前 1 米仅预热灯态感知，车辆继续驶向真实停车点；到点停稳后才进入 `WAIT_TRAFFIC_LIGHT`。
- 稳定绿灯立即放行；连续 15 秒无有效灯态也按比赛恢复策略放行。
- 红绿灯放行时同时启动 GPU YOLO 预加载和前往抓取点，不为模型加载单独停车。
- 抓取纸片阶段只识别相机中心，不左右摆臂；锁定目标类型后保持预抓取观察姿态。
- 实物抓取按中心、左、右各观察一次；识别到后立即执行抓取。前点失败才进入 `RUN_TO_PICKUP_REAR`。
- 前后抓取均失败且比赛配置允许跳过时，进入 `BYPASS_DROP_TASKS`，放弃本环节分数并继续终点；不会假装抓取成功。
- 抓取成功后设置 `has_cargo=true`，夹爪保持闭合，机械臂回统一运输位姿。
- 放置点按中心、左 `+10°`、右 `-10°` 各观察一次。前点失败必须进入后点再观察一轮，不能直接去终点。
- 只有找到放置目标并执行释放后才允许张开夹爪；成功后设置 `has_cargo=false`。
- 避障全程由规划器管理，状态机只记录语义路段，不启停避障。

ArmTask 反馈阶段：

```text
1 MOVING_TO_INSTRUCTION_POSE
2 RECOGNIZING_INSTRUCTION
3 TARGET_TYPE_LOCKED
4 SEARCHING_TARGET_OBJECT
5 OPERATING
6 VERIFYING_OPERATION
```

室内人工展示图片时，`PICKUP_FRONT_TASK` 的 `phase=2` 稳定展示抓取指令图片，出现 `phase=3` 后立即移开。放置阶段保持对应放置图片和放置区域固定，直到 Action 返回结果。

Action 只有在真实完成抓取、夹爪反馈验证、建立持物状态并回到运输位姿后才返回成功。只识别到目标、IK 无解、定位失败、轨迹失败、夹爪未闭合等情况都返回明确失败；持物时拒绝新的 PICKUP。DROP 只有执行释放并验证张开后才成功。

## 11. 完成、人工接管和停机

正常完成必须同时看到：

```text
"state":"FINISHED"
"finished":true
"reason":"finish_arrived_and_stopped"
```

车辆停车后人工接管。在终端 A 按 `Ctrl+C`。若终端断线，先定位唯一 launch PID：

```bash
pgrep -af '/opt/ros/humble/bin/ros2 launch competition_bringup indoor_competition.launch.py'
```

确认后只结束该 PID：

```bash
kill -INT <准确的launch-PID>
```

最后检查：

```bash
pgrep -af 'indoor_competition|mission_node|piper_arm_task_node|ranger_twist_adapter|ranger_base_node' || true
ros2 topic info /cmd_vel
```

相关比赛进程应全部退出，`/cmd_vel` 不应再有比赛节点发布者。保留 `log/competition_manual_*.log`；失败后先保存证据，不连续盲目重启。

## 12. 常见故障定位

### 挥旗成功但车辆不动

先看 `/mission/status` 是否已离开 `WAIT_START_FLAG`，再检查 `/control/state_valid`、`/safety/event`、`/system_state`、`/cmd_vel_safe` 与 `/cmd_vel`。画面显示 `START: CONFIRMED` 只证明感知已确认，不证明底盘 CAN、安全门和速度适配器都已放行。

### 红绿灯预热点停车

正常行为是在预热点继续行驶、真实停车点停车。如果仍在前 1 米停住，记录 `/mission/status`、`/control/status`、`/avoidance/stop_request`、`/safety/event` 和 launch 日志，不手动伪造检查点放行。

### 红绿灯一直 UNKNOWN

查看 `/perception/wrist_traffic_annotated` 是否持续出图、状态是否已进入灯态感知阶段，以及 `/perception/traffic_light_detection` 是否更新。相机图像正常但检测不更新时检查感知节点日志；当前恢复策略会在连续 15 秒无结果后继续。

### rqt 灰白或无话题

先退出 Conda，再确认原始相机话题频率。标注话题只在对应感知节点工作时产生；如果原始话题也无帧，检查 RealSense 是否被重复启动或 USB 设备是否掉线。

### Action server 数量为 2

说明机械臂任务节点重复启动。不要继续比赛，结束准确的旧进程，直到 `/mission/arm_task` 只显示一个 `/piper_arm_task` 服务端。

### Safety 显示 SAFE_HOLD

`remote_not_ready` 通常指底盘遥控/控制模式未就绪；`stale_state` 指底盘状态没有持续更新；`stale_command` 需要结合任务状态判断控制链是否在发布。先恢复底盘和 CAN 状态，不通过手工速度命令绕过 Safety。

## 13. 备份和恢复

本基线有两类备份：

1. Git 标签 `verified/full-flow-slow-20260819`：保存已跟踪、可审查的源码与本文档。
2. 同日期完整运行快照：额外包含当前未纳入 Git 的 Piper SDK、ROS 驱动和迁移运行依赖。

快照明确排除 `.git/`、`build/`、`install/`、`log/`、`recordings/`、`__pycache__/`、`.pytest_cache/` 和 `*.pyc`。运行日志与录包需要按测试轮次单独保存，不能依赖源码快照恢复。

查看 Git 固化点：

```bash
git show verified/full-flow-slow-20260819
```

从 Git 标签导出干净源码时，在电脑端执行：

```powershell
git archive --format=zip --output full-flow-slow-20260819-tracked.zip verified/full-flow-slow-20260819
```

恢复到小车时，仍以电脑端权威副本为源，排除构建和运行产物后定向同步到 `/home/agilex/competition_ws`，再在小车端重新构建。不要把备份中的 `.git`、旧 `install/` 或旧日志覆盖到小车。

## 14. 已验证事实与剩余风险

已验证事实：

- 2026-08-19 操作员在当前室内场地确认了挥旗、红绿灯启停、货架接近、图片/货物识别、抓取、持物行驶、第二货架接近、放置和终点停车的慢速完整闭环。
- PC 自动测试已覆盖状态机主路径、机械臂 Action 成功/失败判定、持物夹爪保护、瓶子/方块策略隔离、前后点恢复和失败不误判成功。
- 小车端已完成针对性测试、Python 编译检查、Shell 语法检查和构建验证。

仍需注意：

- 当前结论来自一次当前场地配置下的完整流程，不代表统计意义上的稳定性；应继续保留每轮日志并统计失败阶段。
- `1.0 m/s` 速度曲线尚未实车验证；已验证完整闭环对应标签内约 `0.119 m/s` 的慢速配置。第一次高速验证必须分阶段逐级放行并检查制动距离、跟踪误差和货物夹持稳定性。
- 正式赛场地图、光照、货物摆放、不同目标类别和网络/USB 波动尚未得到同等覆盖。
- 失败分支虽有自动测试，前后抓取全部失败、前后放置全部失败、红绿灯 15 秒降级和运行中定位恢复仍未全部进行真车组合验证。
- 当前路线是“可完整慢速走通”的冻结基线。后续提速、下探或路线调整应单独提交和验证，不覆盖此标签。
