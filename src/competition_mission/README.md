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

配置入口是 `config/mission/indoor_competition_mission.yaml`。室内整车启动入口是：

```bash
ros2 launch competition_bringup indoor_competition.launch.py
```

该入口默认 `start_base=false`、`start_chassis_adapter=false`、
`start_arm_simulator=false`，因此默认不会驱动车辆或机械臂。实车联调必须继续遵守项目
关于初始位姿、人工确认和运动许可的规则。
