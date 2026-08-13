# scripts

当前已落地脚本：

- `car_source_env.sh`：小车端 ROS/工作空间环境加载辅助脚本。
- `day1_*.sh`：Day1 建图、底盘审计、预检和证据采集脚本。
- `day3_follow_global_plan.py`：Day3 全局路径低速跟踪调试脚本，仅用于现场监督测试。
- `day5_sequential_bringup.sh`：先单独启动并门控 Livox、再启动并门控 FAST-LIO，
  最后加载完整 Day5 导航栈并复检点云；不会启用底盘 relay。
- `day5_sensor_freshness_gate.py`：使用 `KEEP_LAST depth=1` 检查 Livox 扫描末点或 FAST-LIO 点云 P95 年龄。
- `day5_sensor_sync_probe.py`：同时观测 Livox、IMU和 FAST-LIO 最新时间戳、频率与扫描跨度，用于区分传感器源、同步和输出积压。
- `rebuild_fast_lio_release.sh`：幂等应用车端 FAST-LIO 实时性补丁链并执行受限的 Release重建；运行前必须停止 FAST-LIO。
- `day5_record_motion.sh`：按 Day5 权威 topic 清单录包，并用 transient-local QoS 捕获 `/tf_static`。
- `day5_full_route_relay.py`：现场监督 relay；看门狗默认按轨迹 `duration_s * 2.5 + 60s` 计算，显式传入 `--watchdog-timeout-s 0` 可关闭固定总时长，同时保留无进展和其他安全停机条件。
- `day5_run_policy.py`：Day5 轨迹元数据与看门狗纯逻辑。

## Day3 跟踪脚本安全约定

`day3_follow_global_plan.py` 默认发布到非驱动 topic `/cmd_vel_day3_field_test`，避免误触发底盘。

如需监督下直接测试底盘 `/cmd_vel` 通路，必须显式传入：

```bash
python scripts/day3_follow_global_plan.py \
  --plan docs/evidence/day3/debug_global_plan_car.yaml \
  --cmd-topic /cmd_vel \
  --allow-direct-cmd-vel
```

## Day5 顺序启动

以下工具只负责拉起节点并验证传感器新鲜度，不会把 `/cmd_vel_safe`
转发到底盘：

```bash
scripts/day5_sequential_bringup.sh <run_label> \
  trajectory_file:=/home/agilex/competition_ws/docs/evidence/day5/debug_control_validation_trajectory.yaml \
  route_file:=/home/agilex/competition_ws/config/routes/debug_control_validation_route.yaml \
  semantic_map_file:=/home/agilex/competition_ws/maps/debug/semantic_map_control_validation.yaml
```

看到 `DAY5_SENSORS_READY` 后，再另开终端启动证据录包。不要在
FAST-LIO 新鲜度门控前录制高带宽点云：

```bash
scripts/day5_record_motion.sh <run_label>
```

启动底盘、发布初始位姿和运行 relay 仍须遵守现场授权与人工监督规则。
