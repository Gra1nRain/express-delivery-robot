# ROS2 源码区

每个 ROS2 包放在本目录下。包之间依赖方向应保持从接口/基础能力到任务编排，避免循环依赖。

- `competition_localization`：定位链路中的项目自有适配逻辑，例如 FAST-LIO 全局锚定。
- `competition_bringup`：只负责 launch 编排，不承载定位、规划或控制算法实现。
