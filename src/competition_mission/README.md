# competition_mission

负责比赛总状态机、两点装卸分支、红绿灯等待、机械臂 Action 编排和 mission
日志。状态机只读取配置语义，不写死实验室物理坐标。

## 模块

- `mission_state_machine.py`：纯事件状态机。唯一业务接口是
  `CompetitionMissionStateMachine.handle(event)`；不依赖 ROS，可离线测试。
- `mission_node.py`：ROS 适配器。订阅挥旗、红绿灯和控制状态，发布显式停车点
  放行命令，并作为 `ArmTask` Action client。
- `arm_task_simulator_node.py`：无运动的离线 Action 适配器，只用于验证整条状态链，
  不能作为机械臂实机通过证据。
- `arm_task_runner.py`：与 ROS 无关的任务重试、总超时、取消和阶段反馈契约。
- `piper_arm_backend.py`：复用现有 Piper 迁移脚本，但把抓取和放置拆成两个任务；
  原迁移文件不会被改写。
- `piper_arm_task_node.py`：常驻真实 Piper Action server。同一时刻只接受一个任务，
  通过夹爪反馈确认抓取或释放结果。控制器和反馈就绪后会自动保持当前夹爪开度移动到
  统一待机位姿；完成前不接受装卸任务，但不阻止小车等待挥旗或发车。

配置入口是 `config/mission/indoor_competition_mission.yaml`。室内整车启动入口是：

```bash
ros2 launch competition_bringup indoor_competition.launch.py
```

该入口默认 `start_base=false`、`start_chassis_adapter=false`、
`start_real_arm=false`、`start_arm_simulator=false`；共享腕部相机默认常驻，但不会驱动
底盘或机械臂。真实机械臂联调时显式设置 `start_real_arm:=true`，且不得同时启动
`start_arm_simulator`。实车联调必须继续遵守项目关于初始位姿、人工确认和运动许可的
规则。

红绿灯提前 `1.0 m` 的语义标记只开启视觉预热，小车继续驶向真实停止点；到达
`traffic_light_stop_line` 后状态机才独立启用灯态停车约束。绿灯或 `15 s` 无结果
放行时，状态机会先解除停车约束，再关闭红绿灯推理并放行到取货点。
