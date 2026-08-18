# 比赛状态机手动实车测试手册

## 1. 当前事实与边界

- 整车入口是 `competition_bringup/indoor_competition.launch.py`。
- `start_base`、`start_chassis_adapter` 和 `start_real_arm` 默认都是 `false`；只有显式打开相应开关才会连接底盘或真机械臂。
- 状态机启动后停在 `WAIT_START_FLAG`。发布初始位姿不会发车，识别到挥旗或手动发布挥旗事件才会放行路线。
- 共用腕部 RGBD 相机由整车 launch 启动；机械臂适配器使用 `manage_camera=false`，不会再启动第二个相机节点。
- 当前比赛轨迹的名义起点是 `x=-0.376 m`、`y=0.112 m`、`yaw=0.02 rad`。
- 2026-08-18 已在车端完成无底盘、无真机械臂的成功主路径、红绿灯 15 秒无结果放行和装货最终失败继续比赛验证。
- 2026-08-18 操作员已确认真 Piper 的静态“图片识别、实物识别、抓取、保持夹持、图片定位、放置”闭环成功。
- 2026-08-18 已完成无底盘的真 Piper 启动自动待机验证；目标关节位姿到达时最大关节误差为 `0.0010 rad`，随后 ArmTask 正常启用。车端日志为 `log/arm_startup_transit_retry_20260818.log`。
- 尚未验证持物行驶和整条实车运动路线；执行第 4 节前仍应确认第 3 节的软件版本和现场摆放没有变化。

## 2. 每个终端的环境准备

以下命令均在小车端执行。每开一个新终端都先运行：

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh
export CAN_NAME=can2
export PIPER_CAN_NAME=can2
```

Piper 使用 `can2`；Ranger 底盘仍由整车 launch 的控制参数使用 `can3`。不要把两个 CAN 口改成同一个。

启动前确认没有上一轮残留：

```bash
pgrep -af 'indoor_competition|mission_node|piper_arm_task_node|piper_single_ctrl|arm_task_simulator_node|ranger_twist_adapter|ranger_base_node' || true
```

若有残留，先确认它属于上一轮测试，再对准确 PID 发送 `SIGINT`；不要用宽泛的 `pkill`。

启动真机械臂后，软件会等待控制器和反馈就绪，再保持当前夹爪开度自动移动到统一
行驶/待机位姿；该动作不会成为小车挥旗发车的前置判断：

```text
joints_rad: [0.005760, 0.289742, -0.565347, -0.081856, 0.045605, 0.092502]
joints_deg: [0.330, 16.601, -32.392, -4.690, 2.613, 5.300]
```

夹爪开度不属于这个位姿约束。启动回位保持当前开度，抓取后回位保持夹紧，放置后
回位保持张开。启动前必须确认机械臂到该位姿的运动空间无遮挡。

## 3. 第一级：底盘断开的静态机械臂测试

这一阶段会移动机械臂，但不会启动 Ranger 驱动，也不会把速度命令接到 `/cmd_vel`。

### 3.1 启动共享相机和真机械臂

终端 A：

```bash
ros2 launch competition_bringup indoor_competition.launch.py \
  start_base:=false \
  start_chassis_adapter:=false \
  start_wrist_camera:=true \
  start_real_arm:=true \
  start_arm_simulator:=false \
  arm_post_instruction_clear_delay_s:=10.0 \
  rviz:=false
```

`arm_post_instruction_clear_delay_s=10.0` 只用于当前室内人工展示图片的测试：识别并锁定物体类型后，机械臂保持当前姿态 10 秒，让操作员把指令图片移出相机视野。正式比赛默认值为 `0.0`，不会增加等待。

终端 B 检查：

```bash
ros2 action info /mission/arm_task
ros2 node list | sort | grep -E '^/piper_(arm_task|controller)$'
ros2 topic echo /mission/status --once --full-length
ros2 topic info /cmd_vel -v
timeout 5s ros2 topic hz /left_wrist_camera/camera/color/image_raw
```

预期：日志出现 `Startup transit pose reached; ArmTask enabled`；`ros2 action info` 显示
`Action servers: 1` 且服务端节点是 `/piper_arm_task`；节点列表中各有一个
`/piper_arm_task` 和 `/piper_controller`；任务状态是 `WAIT_START_FLAG`，`/cmd_vel`
不存在，相机持续出图。

若需要观察目标是否在画面中央，在小车 Ubuntu 图形桌面的另一个终端运行：

```bash
conda deactivate 2>/dev/null || true
cd /home/agilex/competition_ws
source scripts/car_source_env.sh
ros2 run rqt_image_view rqt_image_view
```

在下拉框选择原始彩色图像 `/left_wrist_camera/camera/color/image_raw`。不要选择 `/perception/wrist_traffic_annotated`，后者是挥旗和红绿灯的标注画面。普通 SSH 没有图形转发时不会显示窗口。

### 3.2 手动执行一次前点抓取

把指令图片和待抓物体放在已测试位置，然后执行：

```bash
ros2 action send_goal --feedback \
  /mission/arm_task competition_interfaces/action/ArmTask \
  "{task_type: 1, station: 1, task_id: manual-pickup-front, target_type_hint: '', max_attempts: 1, timeout_s: 120.0}"
```

反馈阶段应依次包含：

```text
1 MOVING_TO_INSTRUCTION_POSE
2 RECOGNIZING_INSTRUCTION
3 TARGET_TYPE_LOCKED
4 SEARCHING_TARGET_OBJECT
5 OPERATING
6 VERIFYING_OPERATION
```

看到反馈 `phase: 3` 后立即移开指令图片。启用上述 10 秒等待时，下一条 `phase: 4` 会在等待结束后出现，随后机械臂才开始识别实物并抓取。

记录结果中的 `target_type`。只有抓取、夹爪反馈验证和保持夹紧返回统一行驶位姿全部
完成后，Action 才会返回 `outcome: 1`。

### 3.3 手动执行一次放置

下面以抓取结果为 `green_bottle` 为例；必须替换成上一步真实返回的 `target_type`：

```bash
ros2 action send_goal --feedback \
  /mission/arm_task competition_interfaces/action/ArmTask \
  "{task_type: 2, station: 1, task_id: manual-drop-front, target_type_hint: green_bottle, max_attempts: 1, timeout_s: 90.0}"
```

只有放置、夹爪张开反馈验证和保持张开返回统一行驶位姿全部完成后，Action 才会返回
`outcome: 1`。完成后在终端 A 按 `Ctrl+C`，再执行第 6 节的退出检查。

## 4. 第二级：整车全流程实车测试

### 4.1 启动完整比赛流程

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

该命令显式打开了两道底盘运动门：Ranger 驱动和 `/cmd_vel_safe -> /cmd_vel` 适配器。此时状态机仍应等待挥旗，底盘命令应为零。这里的 `10.0 s` 用于室内人工展示并移走抓取指令图片；正式自动比赛时若图片无需人工移走，可恢复默认 `0.0 s`。

### 4.2 发布名义初始位姿

先把小车人工摆正到轨迹起点。终端 B：

```bash
ros2 topic pub --once \
  --wait-matching-subscriptions 3 \
  --keep-alive 2.0 \
  /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -0.376, y: 0.112, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0099998, w: 0.99995}}, covariance: [0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0003046]}}"
```

如果实际摆车位置与名义起点不同，应使用 RViz 的 `2D Pose Estimate` 发布实测位姿，不要照抄上述坐标。

### 4.3 发车前核对

终端 B：

```bash
ros2 topic echo /mission/status --once --full-length
ros2 action info /mission/arm_task
ros2 topic echo /control/state_valid --once
ros2 topic echo /cmd_vel --once
timeout 3s ros2 run tf2_ros tf2_echo map body
```

应看到：

- 任务状态为 `WAIT_START_FLAG`；
- `ros2 action info` 显示 `Action servers: 1`，服务端节点为 `/piper_arm_task`；
- `state_valid=true`；
- `map -> body` 可查询；
- 挥旗前 `/cmd_vel` 的线速度和角速度均为零。

任一项不满足时，不发布挥旗事件，先结束本轮并保留日志。

### 4.4 手动触发发车

完整比赛验证优先使用真实挥旗。若只想验证状态机和整车运动，可在终端 B 手动发布一次等价事件：

```bash
ros2 topic pub --once \
  --qos-durability transient_local \
  /perception/flag_wave_detected std_msgs/msg/Bool \
  "{data: true}"
```

这条命令会真实发车。不要在实车运行时手工发布 `/control/status`、`/mission/checkpoint_release` 或伪造停车点消息。

### 4.5 监视比赛状态

终端 C：

```bash
ros2 topic echo /mission/status --full-length
```

前点装卸都成功时，预期主链为：

```text
WAIT_START_FLAG
RUN_TO_TRAFFIC_STOP
WAIT_TRAFFIC_LIGHT
RUN_TO_PICKUP_FRONT
PICKUP_FRONT_TASK
RUN_TO_DROP_FRONT
DROP_FRONT_TASK
RUN_TO_FINISH
FINISHED
```

前点失败才会进入对应的 `RUN_TO_PICKUP_REAR` 或 `RUN_TO_DROP_REAR`。前后抓取都失败时进入 `BYPASS_DROP_TASKS`，不在卸货点停车而直接继续到终点。红绿灯连续 15 秒没有有效结果时会自动继续。

室内人工图片操作按 `/mission/status` 中的 `arm_feedback.phase` 执行：

- `PICKUP_FRONT_TASK` 的 `phase=2` 时稳定展示抓取指令图片；`phase=3` 后立即移开，10 秒窗口结束后进入实物识别和抓取。
- 只有前点抓取失败才会停车到 `PICKUP_REAR_TASK`；若前点已经锁定类型，后点直接复用该类型，不要再次发送 Action。
- `DROP_FRONT_TASK` 的 `phase=2` 会识别对应卸货图片；保持卸货图片和放置区域固定，直到放置结果返回。
- 只有前点放置失败才会停车到 `DROP_REAR_TASK`。不要手工发布第二点放行消息，分支完全由机械臂结果控制。

## 5. 正常结束判据

`/mission/status` 同时满足以下内容才算状态机完成：

```text
"state":"FINISHED"
"finished":true
"reason":"finish_arrived_and_stopped"
```

到达 `FINISHED` 后小车保持停车，随后由人工接管并结束 launch。

## 6. 结束与残留检查

在终端 A 按 `Ctrl+C`。如果终端断线，先定位唯一的 launch PID：

```bash
pgrep -af '/opt/ros/humble/bin/ros2 launch competition_bringup indoor_competition.launch.py'
```

确认 PID 后执行：

```bash
kill -INT <准确的-launch-PID>
```

最后检查：

```bash
pgrep -af 'indoor_competition|mission_node|piper_arm_task_node|ranger_twist_adapter|ranger_base_node' || true
ros2 topic info /cmd_vel
```

除查询命令自身外不应有相关进程，`/cmd_vel` 不应再有比赛节点发布者。保留本轮 `log/competition_manual_*.log`，失败时不要立即重复启动。

## 7. 尚未验证的风险

- 真机械臂静态抓取和放置已经成功；持物行驶及整车自动触发装卸尚未验证。
- 真机械臂启动自动待机已完成无底盘验证；装卸后的统一行驶位姿返回仍只有 PC 自动测试证据，
  持物行驶尚未验证。
- 删除冗余 ROS 节点名重映射的修复已构建并同步；现场已确认 `/mission/arm_task` 只有一个 Action server。
- 名义初始位姿来自当前轨迹首点，仍需现场摆车或 RViz 校正。
- 红绿灯提前 `1.0 m` 只预热识别、到真实停止点才停车的拆分逻辑已通过 PC
  自动测试，仍需下一轮实车复核；抓取总超时 `120 s` 和放置总超时 `90 s` 是当前比赛参数。
- 当前状态机按既定范围不处理导航不可达；终点停车后由人工接管。
