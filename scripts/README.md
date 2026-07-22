# scripts

当前已落地脚本：

- `car_source_env.sh`：小车端 ROS/工作空间环境加载辅助脚本。
- `day1_*.sh`：Day1 建图、底盘审计、预检和证据采集脚本。
- `day3_follow_global_plan.py`：Day3 全局路径低速跟踪调试脚本，仅用于现场监督测试。

## Day3 跟踪脚本安全约定

`day3_follow_global_plan.py` 默认发布到非驱动 topic `/cmd_vel_day3_field_test`，避免误触发底盘。

如需监督下直接测试底盘 `/cmd_vel` 通路，必须显式传入：

```bash
python scripts/day3_follow_global_plan.py \
  --plan docs/evidence/day3/debug_global_plan_car.yaml \
  --cmd-topic /cmd_vel \
  --allow-direct-cmd-vel
```
