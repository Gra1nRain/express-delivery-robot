# Day 5 启动与静态检查点地图校准实现（2026-08-14）

## 结论

本次实现前两层定位校准，不引入 AMCL，也不实现回环检测：

1. 人工 `/initialpose` 只建立粗略 `map -> camera_init` 锚点；车辆静止后，系统用多帧 `/scan` 对静态 `/map` 的残差自动精修一次。
2. 路线执行到静态检查点并完成停车保持后，再申请一次受门控的地图校准；校准就绪前不允许切换下一段。

本次没有启动底盘、没有发布路线使能，也没有进行实车运动。

功能提交：`ef082e4 feat(localization): add gated map anchor calibration`

## 已实现事实

- `fastlio_anchor_node` 仍是 `map -> camera_init` 唯一 TF 写入者。
- 锚点更新带单调 revision 和 compare-and-swap 检查，旧请求不能覆盖新锚点。
- 启动修正只允许在车辆静止且路线未使能时应用。
- 检查点修正只允许在车辆静止、路线已使能且控制器声明检查点保持时应用。
- 匹配必须连续 5 帧稳定、置信度和中位残差达标、搜索结果不触边；应用后再用 3 帧残差复核。
- 启动修正硬限制为 `0.50 m / 10 deg`，检查点修正硬限制为 `0.20 m / 5 deg`。
- 复核失败会请求回滚；回滚后保持失败闭锁，不自动继续路线。
- 校准状态丢失超过 `1.0 s` 时，控制器请求停车并禁止完成当前段。
- 雷达匹配的 yaw 修正以当前雷达位置为旋转中心；应用到全局锚点前会换算为正确的 map-frame SE(2) 左乘变换。位移安全限幅仍使用车辆当前位置的实际修正量，避免远离地图原点时误判。
- 锚点 revision 变化属于受门控的预期 TF 跳变，控制状态连续性检查会在校准状态回调中显式重置；普通运行中的未知 TF 跳变保护保持不变。

## 无运动验证事实

- 新增并通过启动校准、复核、回滚、运动样本清空、限幅、锁定和写入拒绝测试。
- 新增并通过锚点 revision、回滚、安全上下文、旋转中心换算和检查点完成门禁测试。
- Day 5 启动拓扑测试确认残差节点和校准协调器被启动，且协调器不拥有 TF broadcaster 或速度输出。
- `python -m pytest -q tests`：`300 passed`。
- 提交中的 23 个文件已定向同步到 `/home/agilex/competition_ws`，电脑与小车 SHA-256 全部一致。
- 小车端 `colcon build --symlink-install --packages-select competition_localization competition_control competition_bringup`：3 个包全部成功，耗时 `12.9 s`。
- 小车端新增功能针对性测试：`48 passed in 1.80s`。
- 小车安装空间可发现 `fastlio_anchor_node`、`scan_map_residual_monitor_node`、`startup_alignment_node`；`day5_motion_control.launch.py --show-args` 可见并解析 `start_alignment`、残差参数文件和启动校准参数文件。
- 小车端以 `start_base:=false`、`start_chassis_adapter:=false` 且关闭 Livox、FAST-LIO、近障、局部规划、地图服务的方式短时启动；使用源清单匹配的 `debug_indoor_one_lap_continuous_trajectory.yaml` 后，锚点、残差、校准、MPPI、安全五个节点均成功就绪并在超时后干净退出。
- 实时 `/localization/alignment_status` 为 transient-local / reliable；抽样状态是 `WAITING`、`startup_ready=false`、`anchor_revision=null`，符合“未收到粗锚点时禁止发车”的预期。现场可见 1 个发布者，`fastlio_anchor` 和 `mppi_control` 两个订阅者。
- 第一次使用 launch 的既有默认 `debug_continuous_trajectory.yaml` 时，MPPI 因路线、语义地图、规划和优化源清单 SHA-256 不匹配而按设计拒绝启动；改用最后一次实车方案的匹配 artifact 后通过。未覆盖或重新生成该默认 artifact。
- 直接执行无范围限制的 `python -m pytest -q` 会收集用户已有的未跟踪 Piper ROS 目录，并因本机缺少其 `ament_copyright`、`ament_flake8`、`ament_pep257` 依赖在收集阶段失败；该目录未被修改，也不属于本次受控测试范围。

## 尚未验证与风险

- 尚未启动 Livox/FAST-LIO 传感器链，未发布 `/initialpose`，因此尚未验证从粗锚点到自动精修、复核和 READY 的完整静止 topic/TF 闭环。
- 尚未用实车静止扫描确认 5 帧候选和 3 帧复核在现场几何下可稳定收敛。
- 尚未验证检查点处地图几何是否都足够可观；低置信度时系统会安全地持续停车，而不是猜测修正。
- 尚未进行任何校准后的自主路线测试。下一次实车运动仍必须由用户重新发布初始位姿并明确允许发车。
