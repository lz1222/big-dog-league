# RK Inspection ROS 2 Workspace

2026 睿抗 CAIR 强体赛道足式机器人多模态巡检项目。当前仓库同时包含真实硬件子系统、mock 集成节点、硬件调试工具和未完成 adapter；不能把“能编译”或“mock 全流程成功”解释为国赛评分功能已经完成。

本次 `National Competition Baseline Audit` 的仓库实况见：

- [170 分评分矩阵](docs/NATIONAL_SCORE_MATRIX.md)
- [当前实际架构](docs/ARCHITECTURE_CURRENT.md)
- [国赛阻塞项](docs/NATIONAL_BLOCKERS.md)

当前没有任何一个 170 分场地项具备本仓库内可追溯的硬件验收证据，也没有真实 170 分完整比赛 launch。

## 环境边界

| 环境 | ROS 2 | 用途 | 结论边界 |
| --- | --- | --- | --- |
| 开发虚拟机 | Humble | 编辑、静态检查、单测、尽可能构建 | VM 成功不证明机器人 Foxy 或硬件动作成功 |
| Unitree Go2 机器人 | Foxy | 最终构建、topic/action 联调、SDK/UDP 和硬件验收 | 机器人端结果必须记录代码提交、参数、命令和日志 |

不要用 Humble 上的依赖或构建问题直接判断 Foxy 机器人必然失败；也不要用 Humble 构建成功替代 Foxy 验证。源码在 VM 修改后，应同步 `src/`、`docs/` 和必要脚本到机器人工作区，由机器人 Foxy 重新构建，不要复制 VM 的 `build/`、`install/` 或 `log/`。

## 构建

### 机器人 / Foxy

```bash
cd ~/rk_inspection_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### VM / Humble

当前 checkout 可能位于非 ASCII 路径。若 `rosidl` 在生成 `rk_interfaces` 时受路径影响，可把 build/log 放在 ASCII 临时路径：

```bash
cd ~/rk_inspection_ws
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/rk_inspection_log build --symlink-install \
  --build-base /tmp/rk_inspection_build \
  --install-base /tmp/rk_inspection_install
source /tmp/rk_inspection_install/setup.bash
```

现有 helper 会尝试 rosdep 后走标准构建：

```bash
./scripts/build_all.sh
```

## 启动类型：不要混用

### 1. Mock 全流程

```bash
source install/setup.bash
ros2 launch rk_bringup mock_competition.launch.py
```

该 launch 启动 mock line/sign/item perception、line follower、line-course mission、mock locomotion、mock arm、mock safety 和完整 mission state machine。默认 `auto_start=true`。

手动触发 mock mission：

```bash
ros2 launch rk_bringup mock_competition.launch.py auto_start:=false
ros2 run rk_tools mission_client_node
```

这条链不驱动真实 Go2 或机械臂。mock action server 对任意动作固定等待后成功，因此不能作为起跳、台阶、抓放、警示动作或停靠的完成证据。

### 2. 真实巡线比赛子系统（不是 170 分完整启动）

机器人 Foxy 环境中：

```bash
source install/setup.bash
ros2 launch rk_bringup competition_line_nav.launch.py
```

它实际启动 D435i、真实 line tracker、line follower、line-course final controller、SDK UDP forwarder 和仓库外 `go2_sdk_udp_server`，并可切换 EconomicGait。机器人等待独立 `/mission/start`：

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'
```

该 launch **不启动**：

- 完整 `mission_state_machine_node`
- 真实 `/locomotion/execute_motion` server
- 任一 arm action server
- `real_sign_detector_node`
- 真实 item-tag/object-position producer
- 白横线动作执行者

因此它只能称为“真实巡线/line-course 子系统”，不能称为国赛完整任务。

SSH 安全的后台启动也保持“节点启动”和“任务开始”分离：

```bash
src/rk_bringup/scripts/start_line_system.sh
src/rk_bringup/scripts/mission_start.sh
```

停止：

```bash
src/rk_bringup/scripts/mission_stop.sh
src/rk_bringup/scripts/stop_line_system.sh
```

### 3. 独立硬件调试入口

- `sign_action_debug.launch.py`：真实 sign + 警示动作调试，不是完整路线。
- `obstacle_practical.launch.py`：gait 避障 + depth/bridge 调试，不是完整 mission。
- `obstacle_direct_open_loop.launch.py`：独立硬编码 route 工具，不是完整比赛状态机。
- `arm_task.launch.py`：旧 DryRun fixed-pose arm。
- `d1_pick.launch.py`：默认 DryRun 的 D1 topic-only 路径。
- `new_arm_task.launch.py`：只把 JSON 发到尚无仓内订阅者/ACK 的 new-arm bridge topic。

这些入口必须单独运行。不要与主巡线、keyboard route、speed sweep 或 two-step test 同时发布 `/navigation/cmd_vel`。

## 运行前检查

```bash
./scripts/national_preflight.sh
```

脚本明确输出 `PASS/WARN/FAIL`，检查：

- 本次改动是否包含 `build/install/log`、cache、录制媒体或大调试图片
- 历史已跟踪生成物
- `package.xml` / `setup.py` 基本一致性和 console target
- launch 中的内部 executable
- YAML 解析和空配置
- Python 语法
- 关键 ROS message/action
- 合并冲突标记

当前基线预期会因历史已跟踪的 `third_party` build/install/log 与 cache/venv 文件返回 FAIL；这是真实审计结果，不能通过忽略输出伪装成 PASS。清理 vendor/generated 历史应另开范围清晰的变更，避免误删 SDK 必需文件。

正式机器人启动后还必须做运行时所有权检查：

```bash
ros2 topic info -v /navigation/cmd_vel
ros2 topic info -v /perception/line_track
ros2 topic hz /perception/line_track
ros2 action list
```

正常比赛组合中，最终 `/navigation/cmd_vel` 必须只有一个预期 publisher。

## 关键接口（冻结）

- `/perception/line_track`: `rk_interfaces/msg/LineTrack`
- `/perception/sign_detections`: `rk_interfaces/msg/SignDetectionArray`
- `/perception/item_tags`: `rk_interfaces/msg/ItemTagArray`
- `/navigation/cmd_vel`: `geometry_msgs/msg/Twist`
- `/gait/control_lock`: `std_msgs/msg/Bool`
- `/locomotion/execute_motion`: `rk_interfaces/action/ExecuteMotion`
- `/arm/execute_task`: `rk_interfaces/action/ExecuteArmTask`
- `/mission/run`: `rk_interfaces/action/RunMission`
- `/safety/estop`: `std_srvs/srv/SetBool`

本轮审计没有重命名或修改上述 topic、message、service、action 或 console entry point。

## 基础验证

推荐在干净环境中执行并保留完整输出：

```bash
python3 -m compileall src
pytest
colcon build --symlink-install
```

注意：`compileall` 和 `pytest` 可能生成已忽略的 cache；提交前仍要运行 preflight 和 `git status --short`。Humble 构建建议使用 `/tmp` build/install/log base，机器人端必须再用 Foxy 构建和运行验证。无法运行的检查应在 PR 中写明原因，不得填成成功。

## 文档新旧状态

保留旧文档是为了追溯，不代表其中描述仍是当前事实。

### 当前基线文档

- `docs/NATIONAL_SCORE_MATRIX.md`
- `docs/ARCHITECTURE_CURRENT.md`
- `docs/NATIONAL_BLOCKERS.md`
- `src/rk_bringup/README_line_system.md`（当前 line-course 子系统操作说明；仍需以源码和机器人参数为准）

### 规划文档：不能作为实现证明

- `docs/three_person_development_plan.md`
- `docs/race_route_plan.md`
- `docs/code_integration_plan.md`

其中部分“未来实现”描述已经与当前代码进度不一致，例如 locomotion action server 和 real sign detector 已有源码；“完整真实比赛 launch 缺失”等结论仍成立。使用时必须和本次基线文档交叉检查。

### 过时的 stage-one / 历史分析材料

- `docs/race_checklist.md`
- `docs/robot_network_setup.md`
- `docs/unitree_api_message_setup.md`
- `README_ANALYSIS.md`
- `SYSTEM_ANALYSIS_REPORT.md`
- `DEPENDENCY_ANALYSIS.md`
- `DETAILED_DATAFLOW.md`
- `QUICK_REFERENCE.md`
- `ANALYSIS_COMPLETION_REPORT.md`

这些文件可能仍称仓库为纯 mock、只列早期 launch，或引用旧参数。它们没有删除，但不应作为当前国赛启动、接口所有权或功能完成性的依据。

## 提交物策略

不要提交：

- `build/`、`install/`、`log/`
- `__pycache__/`、`.pytest_cache/`、`.pyc`、本地虚拟环境
- rosbag、数据库和录制视频
- 大体积 debug 图片或临时抓图

仓库历史中已经存在部分已跟踪 vendor build/install/log 和 cache 文件；本轮审计不做高风险批量删除，也不新增这些产物。
