# competition_avoidance

新增的静态/动态避障模块。它不修改现有规划、控制或安全实现，只在已经冻结的
`/avoidance/*` seam 上提供一个可替换的 ROS adapter。

## 数据流

```text
/cloud_registered_body + /odom + map<-body TF
  -> 点云 ROI/体素/聚类
  -> 稳定最近邻 + alpha-beta 速度跟踪
  -> 静态/动态滞回分类
  -> 二维相对状态 CPA 风险
  -> /avoidance/objects
  -> /avoidance/status
  -> /avoidance/corridor_update
  -> /avoidance/local_costmap
  -> /avoidance/stop_request
```

现有 `LocalTrajectoryPlanner` 继续订阅 `/avoidance/local_costmap`，现有
`SafetySupervisor` 继续订阅 `/avoidance/stop_request`。本包不发布
`/planning/local_trajectory`、`/cmd_vel_safe` 或 `/cmd_vel`。

## 保守参数

`config/avoidance/avoidance_params.yaml` 继承当前全局规划和 Day5 低速约束：

- 规划最小转弯半径：`0.81 m`
- 最大速度：`0.15 m/s`
- 最大加速度：`0.20 m/s^2`
- 最大减速度配置值：`0.30 m/s^2`
- 近场停车距离：`0.85 m`
- 动态风险安全余量：`0.40 m`

## 安全运行

`vehicle_avoidance_bringup.launch.py` 当前只允许 dry-run。它固定：

```text
start_base:=false
start_chassis_adapter:=false
start_proximity_stop:=false
command_output_topic:=/cmd_vel_safe
```

旧 `proximity_stop_node` 的源码仍保留；dry-run 运行配置只选择本包作为规范
`/avoidance/local_costmap` 和 `/avoidance/stop_request` 发布者，避免多发布者冲突。

## 已验证事实

- 纯 Python 点云聚类、跟踪、CPA/TTC 风险和决策接口有 PC 单元测试。
- 架构测试保证新增节点不拥有 `/cmd_vel` 或 `/planning/local_trajectory`。
- 点云、里程计、TF 或时间戳异常时，模块持续发布停车请求。

## 未验证

- `ground_max_z_m=-0.20` 尚未用本车实测点云冻结。
- 车长/车宽配置 `0.72/0.50 m` 的权威来源仍需手册/CAD或现场测量确认。
- 动态人员分类目前是几何候选，不是视觉语义识别。
- `/avoidance/corridor_update` 当前是 JSON 状态通知，正式 polygon 消息尚未冻结。
- 锥桶主动变道继续依赖现有局部规划器，尚未完成 rosbag 和实车验收。
- 最大减速度 `0.30 m/s^2` 是软件配置值，不是实测制动能力。
