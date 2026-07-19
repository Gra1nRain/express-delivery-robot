# 比赛项目开发手册

本文件记录本项目可复用的远程开发方案、操作习惯、验证入口和风险边界。新经验先记录为“未验证”，完成实际验证后再改成“事实”或“建议”。

## 1. 当前远程开发架构

```text
Windows 开发电脑
  ├─ Codex 修改 competition_car
  ├─ Git 管理本地提交
  ├─ GitHub 私有仓库备份
  └─ Tailscale 网络 + SSH 命令通道
          ↓
Ubuntu 22.04 小车工控机
  ├─ ~/competition_ws       比赛项目工作区
  ├─ ~/agilex_ws             硬件依赖工作区
  └─ 负责构建、运行、录包和读取实车日志
```

### 已验证事实

- 校园网会阻断两台设备之间的直接 SSH 连接。
- 电脑和工控机已加入同一个 Tailscale 网络。
- 当前稳定连接入口是电脑上的 SSH 别名：

  ```powershell
  ssh ranger-mini
  ```

- 小车端 SSH 用户是 `agilex`，主机名当前为 `ubuntu`。
- 小车端系统是 Ubuntu 22.04 + ROS2 Humble。
- 比赛工作区是 `/home/agilex/competition_ws`。
- 小车已有硬件依赖工作区 `/home/agilex/agilex_ws`。
- `scripts/car_source_env.sh` 会依次加载 ROS2、硬件依赖工作区和比赛工作区环境。
- GitHub 私有仓库是 `https://github.com/Gra1nRain/express-delivery-robot`。
- Git 和 GitHub 只在开发电脑上管理；小车不保存项目 Git 历史，也不需要登录 Codex 或 GitHub。

## 2. 远程连接操作

### 2.1 连接前检查

在电脑上确认小车开机并联网，然后执行：

```powershell
ssh ranger-mini "hostname; id -un; pwd"
```

预期结果包含：

```text
ubuntu
agilex
/home/agilex
```

如果失败，按以下顺序检查：

1. 小车是否联网。
2. 两台设备是否都登录同一个 Tailscale 网络。
3. `ssh ranger-mini` 是否仍指向正确的 Tailscale 主机名/地址。
4. 小车 SSH 服务是否运行。
5. 最后再检查用户认证，不要先改代码或反复重试命令。

### 2.2 远程执行命令

单条只读检查：

```powershell
ssh ranger-mini "hostname; ros2 --version"
```

加载项目环境后执行 ROS2 命令：

```powershell
ssh ranger-mini "bash -lc 'source ~/competition_ws/scripts/car_source_env.sh && ros2 topic list'"
```

### 2.3 构建和运行原则

默认先执行只读预检查，再构建：

```powershell
ssh ranger-mini "bash -lc 'source ~/competition_ws/scripts/car_source_env.sh && cd ~/competition_ws && colcon build --symlink-install'"
```

构建后再启动具体 launch。涉及底盘、CAN、运动或机械臂时，必须先确认：

- 急停有效；
- 遥控器可接管；
- 场地和车轮安全；
- 当前 launch 是否会发送运动命令；
- 日志和 rosbag 输出目录已确定。

Day 1 建图入口默认不发送运动目标。需要启用底盘时，必须由操作员明确确认 `start_base`、CAN 端口和现场安全状态。

## 3. 文件同步约定

### 当前事实

- 本项目源代码的权威副本在电脑端：

  ```text
  E:\Myself\Project\无人系统大赛\competition_car
  ```

- 小车端运行副本在 `/home/agilex/competition_ws`。
- 当前已完成过手动同步；自动同步工具尚未正式配置。

### 计划约定

后续自动同步只发送源代码、配置、启动文件和脚本。以下内容不应同步：

```text
.git/
build/
install/
log/
recordings/
*.db3
*.bag
*.bag.active
```

`.stignore` 已预留这些排除规则。自动同步正式启用前，必须先确认：

1. 电脑端目录是发送端；
2. 小车端目录是接收端；
3. 小车端额外生成的文件不会反向覆盖电脑端；
4. 同步完成后才开始构建；
5. 同步失败时保留旧的可运行版本和日志。

## 4. Git 与 GitHub 约定

### 日常流程

```text
电脑端修改
→ git diff / 本地检查
→ 同步到小车
→ 小车端构建和实车验证
→ 保存日志/证据
→ 电脑端 git commit
→ 推送 GitHub 私有仓库
```

常用检查：

```powershell
git status --short --branch
git diff --stat
git log -3 --oneline --decorate
```

提交前确认：

- 没有把 Token、密码、私钥、`.env` 或本地凭据加入提交；
- 没有把 `build/`、`install/`、`log/`、录包加入提交；
- 只提交当前任务范围内的文件；
- 提交说明能够说明改了什么、为什么改、如何验证。

### 回滚原则

- 未验证的新改动先保留在本地分支或工作区，不直接覆盖小车上的最后可运行版本。
- 小车运行版本应能通过目录、提交号或时间戳追溯。
- 回滚前先保存当前日志和差异，不使用破坏性重置命令覆盖用户改动。

## 5. 可复用的开发习惯

### 5.1 一次只改一个变量

例如调控制器时，不要同时修改速度、曲率权重、轨迹 horizon 和底盘模式。每次只改一个参数，保存测试证据，再决定下一步。

### 5.2 先证据，后判断

遇到“车不动、定位跳、规划失败或节点异常”时，先保存：

- 命令原文；
- stdout/stderr；
- topic/TF 快照；
- 参数文件版本；
- rosbag、截图或视频；
- 修改前后的 Git diff。

不要在没有日志的情况下连续盲目重试。

### 5.3 把实验室和正式场地分开

- 实验室使用 `debug_site_profile.yaml` 和 `debug_route.yaml`。
- 正式场地使用 `competition_site_profile.yaml` 和 `competition_route.yaml`。
- 临时物理摆放不能改变代码中的官方语义，例如 `CONE_LANE_CHANGE`、`DOCK(DROP)` 和 `ARM_TASK(DROP)`。
- 现场只替换地图、语义点、路线点、ROI、dock pose、限速和小范围阈值，不重写主状态机和接口协议。

### 5.4 安全层必须是最后出口

普通规划、轨迹和任务节点不能绕过安全层直接控制底盘。任何传感器断流、规划超时、轨迹越界、超速或避障停车请求，都必须能进入 `SAFE_HOLD` 或最终安全停车出口。

### 5.5 机械臂动作期间保持底盘静止

小车精停成功后先锁定底盘，再发送机械臂 action。机械臂返回 success、failed 或 timeout 都必须有明确状态转换和日志。

### 5.6 文档区分信息性质

建议在文档和日志中使用以下标签：

- **事实**：已经在当前设备和当前版本验证过。
- **经验**：过去有效，但仍需确认是否适用于当前场地或版本。
- **建议**：设计方向，尚未通过实车验收。
- **未验证**：尚未有足够证据，不能当作可用能力。

## 6. 后续经验记录模板

以后遇到可复用的技巧，在本文件追加：

```text
### YYYY-MM-DD：简短标题

性质：事实 / 经验 / 建议 / 未验证
场景：
问题：
做法：
验证：
风险或回滚：
```

## 7. 当前未验证项

- 自动同步工具尚未完成端到端验证。
- Tailscale 当前连接可能经由中继，延迟会随网络变化。
- RANGER 是否支持四轮转角/轮速接口仍需实车确认。
- `/cmd_vel.linear.y` 是否真实生效仍需监督下低速测试。
- 自定义 ROS2 interface 字段和机械臂 action 协议尚未冻结。
- 正式比赛地图、语义点、视觉 ROI 和 dock pose 尚未采集。
