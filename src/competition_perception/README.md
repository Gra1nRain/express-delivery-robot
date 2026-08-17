# competition_perception

本包复用机械臂左腕 RealSense 的彩色图像话题，同时完成：

- HSV 红旗区域追踪和“挥下”动作状态机；
- YOLO 交通灯识别，类别以权重内嵌的 `red/green/off/yellow` 为准；
- `/perception/traffic_stop_request` 安全停车请求。

节点不会再次打开 `/dev/video*`，也不会启动机械臂或底盘。它订阅现有
`/left_wrist_camera/camera/color/image_raw`，因此可以与此前的抓取视觉共享相机。

常驻启动入口只绑定机械臂腕部 D435 的稳定 USB 口 `2-3.3.2`，不会启动或使用
前向相机：

```bash
ros2 launch competition_perception wrist_traffic.launch.py
```

该 launch 同时启动腕部 RGBD 相机和交通感知。后续执行
`Piper_Grasp_Humble_Migration_20260723/run_grasp_single.sh` 时，脚本会检查 RGB、
对齐深度和 camera_info 是否都有发布者；三路健康时保留相机并直接复用。

规则为：未识别到挥旗时停车；挥旗下压确认后允许发车；红灯、黄灯和灭灯停车；
连续确认绿灯后恢复；相机断流时停车。交通灯默认连续 3 帧确认，避免单帧误检。

如果常驻感知已经运行，比赛模式只需要求安全层接收它的停车请求，不要重复启动
第二个感知节点：

```bash
ros2 launch competition_bringup day5_motion_control.launch.py \
  require_traffic_rules:=true
```

上述命令默认仍不会启动底盘驱动或最终 `/cmd_vel` 适配器；实际运动仍需按项目
实车规则另行明确开启。运行前必须已有腕部相机话题。参数在
`config/perception/wrist_traffic_rules.yaml`。

运行环境需要 ROS 2 的 `cv_bridge`，以及与 ROS Python 兼容的 `opencv`、`numpy`
和 `ultralytics`。小车 `/usr/bin/python3` 已只读验证可导入这些依赖；PC 当前未
安装 `ultralytics`，所以 PC 端只执行状态机和安全层测试，不宣称完成 PC 推理。

事实：交付权重 SHA-256 为
`6b7fc4adf8af46500d10666d7c5917b3a98a627353b8aa07bbe4155f04f172bb`。
小车原目录的 `data/traffic_light.yaml` 是自训练三类配置，类别顺序与本权重不同，
运行时不得用它覆盖权重内嵌类别。
