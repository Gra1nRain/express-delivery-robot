# launch

当前已落地的启动入口：

```bash
ros2 launch competition_bringup day1_mapping.launch.py
```

计划拆分：

```text
bringup.launch.py
├─ sensors.launch.py
├─ fast_lio.launch.py
├─ map_and_semantic.launch.py
├─ planning.launch.py
├─ trajectory_optimization.launch.py
├─ control.launch.py
├─ avoidance_interface.launch.py
├─ perception.launch.py
├─ mission_manager.launch.py
├─ docking.launch.py
├─ arm_bridge.launch.py
└─ safety.launch.py
```
