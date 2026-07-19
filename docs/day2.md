# Day 2 语义地图与相机确认

## 目标

Day 2 使用 Day 1 选定的 `maps/debug/map.yaml` 和实际有效区域，完成第一版语义地图、路线点和视觉设备命名确认。不要重新建图，除非 Nav2 加载后证明当前地图不可用。

## 已确认硬件事实

| 设备 | 名称 | 序列号 / 接口 | 状态 |
|---|---|---|---|
| 车上 D435 | `front_camera` | `236223021647` | USB 已枚举 |
| 机械臂腕部 D435 | `left_wrist_camera` | `152223024925` | USB 已枚举 |
| 机械臂 CAN | `can2` | 1 Mbps | 被动监听有反馈 |
| 底盘 CAN | `can3` | 500 kbps | Day 1 建图已使用 |

## 相机约束

- 硬件身份按 RealSense serial 记录。
- 当前小车端 `realsense2_camera` 使用 `serial_no` 参数未能匹配设备；实测可用的 ROS 选择方式是 `usb_port_id`。
- 不允许用 `/dev/video0`、`/dev/video6` 这类编号区分相机。
- 车上视觉任务默认使用 `front_camera`。
- 机械臂取放货感知默认使用 `left_wrist_camera`。
- 旧 Piper 脚本中的默认腕部相机序列号 `151222079131` 与当前硬件不一致，后续接入时必须改为 `152223024925` 或通过环境变量覆盖。

当前验证过的 ROS 启动选择器：

```text
front_camera: usb_port_id=2-3.1.1.1
left_wrist_camera: usb_port_id=2-3.3.2
```

已验证两台相机可以同时启动，并发布 color 与 aligned depth topic。点云 topic、相机 TF 和机械臂手眼坐标仍未验证。

## Day 2 输出

```text
maps/debug/semantic_map.yaml
config/routes/debug_route.yaml
config/perception/debug_vision_rois.yaml
docs/day2.md
```

语义地图至少记录：起点、有效行驶区域、障碍区边界、取货点、投放点、终点停车区和路线中心线草案。
