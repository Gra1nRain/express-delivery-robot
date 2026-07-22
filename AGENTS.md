# competition_car 项目约定

## 范围

本文件只约束 `competition_car/`。仓库上级的 `AGENTS.md` 仍然适用。

## 工作区与同步

- Git 只在电脑端维护，小车端不保存项目 Git 历史。
- 同步到小车不需要每次再经过用户同意；完成一个已提交、已验证的工作片段后，可以主动从电脑端将相关内容定向同步到 `/home/agilex/competition_ws`。
- 远程同步默认不在小车端创建额外备份；只有用户明确要求时才备份。
- 同步排除 `.git/`、`build/`、`install/`、`log/`、`recordings/` 和缓存。
- 小车端使用独立工作区，不覆盖旧 demo 或现有 `agilex_ws`。
- 同步方向固定为电脑端权威副本到小车端运行副本，不执行反向覆盖。

## 架构约束

- 路线顺序、场地坐标、停车位、视觉 ROI、限速和超时必须配置化。
- 状态机只处理语义 step，不判断“实验室第几段路”。
- 普通控制链路必须保持 `global_path -> optimized_trajectory -> body_cmd -> safety -> chassis`。
- 任何最终底盘速度都必须经过安全层；不允许普通节点绕过安全出口。
- 避障组通过 `config/avoidance/` 约定接口接入，不在主控包中复制避障组算法。
- 机械臂动作期间底盘必须锁定。

## 验证入口

ROS2 环境和依赖尚未在本电脑端验证。小车端初始验证命令预计为：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

执行前必须确认小车端 ROS2 发行版、工作区路径和依赖包状态。
