# competition_avoidance

新增的静态/动态避障模块。它不修改现有规划、控制或安全实现，只在已经冻结的
`/avoidance/*` seam 上提供一个可替换的 ROS adapter。

## 数据流

```text
/cloud_registered_body
  -> 现有 proximity_stop
  -> /avoidance/scan (Live Scan，唯一发布者)
  -> avoidance_manager + /odom + map<-body TF
  -> LaserScan 平面点转换/聚类
  -> 稳定最近邻 + alpha-beta 速度跟踪
  -> 静态/动态滞回分类
  -> 二维相对状态 CPA 风险
  -> /avoidance/objects
  -> /avoidance/status
  -> /avoidance/corridor_update
  -> /avoidance/local_costmap
  -> /avoidance/stop_request
```

`avoidance_manager` 不再重复订阅大型 3D 点云，也不发布
`/avoidance/scan`。二维聚类使用 `SCAN_CANDIDATE` 标签，仅表示可参与连续运动
判定，不表示已经识别人或锥桶。

现有 `LocalTrajectoryPlanner` 继续订阅 `/avoidance/local_costmap`，现有
`SafetySupervisor` 继续订阅 `/avoidance/stop_request`。本包不发布
`/planning/local_trajectory`、`/cmd_vel_safe` 或 `/cmd_vel`。

## 保守参数

`config/avoidance/avoidance_params.yaml` 继承当前全局规划和 Day5 低速约束：

- 规划最小转弯半径：`0.81 m`
- 最大速度：`0.15 m/s`
- 最大加速度：`0.20 m/s^2`
- 最大减速度配置值：`0.30 m/s^2`
- 近场停车距离：`1.20 m`
- 动态风险安全余量：`0.40 m`

## 安全运行

`vehicle_avoidance_bringup.launch.py` 当前只允许 dry-run，并且只启动
`avoidance_manager`。它不 include 整套 Day5，因此只能在人工确认传感器、
定位、规划、控制和安全链均已就绪且没有重复发布者后手动叠加；它不会启动
Livox、FAST-LIO、MPPI 或 Safety。

发布初始位姿并完成人工拓扑检查后，直接启动：

```bash
source /home/agilex/competition_ws/scripts/car_source_env.sh
ros2 launch competition_avoidance vehicle_avoidance_bringup.launch.py \
  dry_run:=true \
  enable_chassis_output:=false \
  operation_mode:=dry_run \
  avoidance_params_file:=/home/agilex/competition_ws/config/avoidance/avoidance_params.yaml
```

Day5 必须使用
`config/safety/safety_params_day5_avoidance_scan.yaml` 启动原
`proximity_stop`。该配置只把它的停车请求和代价地图改到诊断话题；Live Scan
仍保持 `/avoidance/scan`。这样规范 `/avoidance/stop_request` 和
`/avoidance/local_costmap` 仍各只有 `avoidance_manager` 一个发布者。

启动前必须人工检查 `/odom`、`/cloud_registered_body`、`/avoidance/scan`、
`/avoidance/local_costmap`、`/avoidance/stop_request`、
`/planning/local_trajectory`、`/cmd_vel_safe` 和 `/cmd_vel` 的发布者数量。
不得使用基础 `safety_params.yaml` 同时启动两者，否则会造成规范话题重复发布。

`/avoidance/local_costmap` 在 scan 对应的 TF 时间戳下直接锚到 `map`，避免
局部规划器用最新 TF 转换旧 body 栅格。RViz 继续直接显示原
`/avoidance/scan`。

## 已验证事实

- LaserScan 转平面点、聚类、跟踪、CPA/TTC 风险和决策接口有 PC 单元测试。
- 架构测试保证新增节点不拥有 `/cmd_vel` 或 `/planning/local_trajectory`。
- Live Scan、里程计、TF 或时间戳异常时，模块持续发布停车请求。
- `/odom` 的 `header.frame_id` 与运行配置不一致时，模块 fail-closed。

## 未验证

- `ground_max_z_m=-0.20` 尚未用本车实测点云冻结。
- 车长/车宽配置 `0.72/0.50 m` 的权威来源仍需手册/CAD或现场测量确认。
- 2D scan 只能做连续运动分类，不具备人员/锥桶语义识别。
- `/avoidance/corridor_update` 当前是 JSON 状态通知，正式 polygon 消息尚未冻结。
- 锥桶主动变道继续依赖现有局部规划器，尚未完成 rosbag 和实车验收。
- 最大减速度 `0.30 m/s^2` 是软件配置值，不是实测制动能力。
