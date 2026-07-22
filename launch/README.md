# launch

当前已落地的启动入口：

```bash
ros2 launch competition_bringup day1_mapping.launch.py
ros2 launch competition_bringup day3_global_planning.launch.py
ros2 launch competition_bringup day3_navigation.launch.py
```

## Day3 启动约定

- `day3_global_planning.launch.py`：只启动地图/路线/全局路径可视化发布，不启动底盘。
- `day3_navigation.launch.py`：启动 Day3 定位与规划链路，`start_base` 默认是 `false`。
- 只有现场确认 CAN、急停/安全出口、停车行为后，才允许显式加 `start_base:=true` 启动底盘驱动。
