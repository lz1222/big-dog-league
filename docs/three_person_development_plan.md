# 三人并行开发代码分工

本文档依据《睿抗四足多模态巡检项目三人工作分工说明》和当前 `src` 代码状态整理，目标是让三名成员可以按文件边界并行推进，减少互相覆盖和合并冲突。

## 总体链路

```text
RealSense RGB image
-> rk_perception/real_line_tracker_node.py
-> /perception/line_track
-> rk_navigation/line_follower_node.py
-> /navigation/cmd_vel
-> rk_unitree_driver 或 rk_go2_sdk_bridge
-> Go2 运动

rk_mission/mission_state_machine_node.py
-> /mission/start, /mission/stop
-> /locomotion/execute_motion
-> /arm/execute_task
```

核心原则：

- 不改公共接口名，特别是 `/perception/line_track`、`/navigation/cmd_vel`、`/mission/start`、`/mission/stop`、`/locomotion/execute_motion`、`/arm/execute_task`、`/mission/run`。
- 不多人同时修改同一个核心节点。`line_follower_node.py`、`gait_control_node.py`、`mission_state_machine_node.py` 分别由不同成员负责。
- 成员 1 只负责看线和沿线走，成员 2 只负责 Go2 动作和安全，成员 3 只负责比赛流程、机械臂和识别映射。
- 合并前必须本地 `colcon build --symlink-install` 通过，不能提交 merge conflict 标记。

## 成员 1：巡线视觉与导航

职责：让机器人看得见黑线、沿线走、丢线不乱冲、终点能停。

主负责文件：

- `src/rk_perception/rk_perception/real_line_tracker_node.py`
- `src/rk_perception/config/perception.yaml`
- `src/rk_perception/launch/perception.launch.py`
- `src/rk_navigation/rk_navigation/line_follower_node.py`
- `src/rk_navigation/config/navigation.yaml`
- `src/rk_bringup/config/line_nav_params.yaml`
- `src/rk_bringup/launch/competition_line_nav.launch.py`
- `src/rk_bringup/launch/vision_nav_debug.launch.py`
- `src/rk_bringup/scripts/check_line_system.sh`
- `src/rk_bringup/scripts/view_line_debug.sh`

优先任务：

| 优先级 | 任务 | 验收标准 |
| --- | --- | --- |
| P0 | 保证导航节点无合并冲突、可启动 | `py_compile` 通过，无 `<<<<<<<` 等标记 |
| P0 | 跑通 `/perception/line_track` 到 `/navigation/cmd_vel` | 能 echo 到 LineTrack 和 Twist |
| P1 | 调真实黑线识别参数 | 直线、弯道、90 度弯稳定 `line_visible=true` |
| P1 | 调丢线和搜索重获策略 | 短时丢线不乱冲，长时丢线停或低速搜索 |
| P2 | 终点停靠辅助 | 回到启停区时能稳定停止 |

成员 1 对外只承诺：

- 输入 `/mission/start`：开始巡线导航。
- 输入 `/mission/stop`：停止巡线导航并发布零速度。
- 输入 `/gait/control_lock`：成员 2 固定动作占用控制权时，导航节点暂停发布 `/navigation/cmd_vel`。
- 输出 `/perception/line_track`：视觉巡线结果。
- 输出 `/navigation/cmd_vel`：导航速度。

建议自测命令：

```bash
python3 -m py_compile src/rk_navigation/rk_navigation/line_follower_node.py
python3 -m py_compile src/rk_perception/rk_perception/real_line_tracker_node.py
PYTHONPATH=src/rk_perception:$PYTHONPATH python3 -m pytest src/rk_perception/test/test_real_line_tracker_node.py
ros2 launch rk_bringup vision_nav_debug.launch.py
ros2 topic echo /perception/line_track
ros2 topic echo /navigation/cmd_vel
```

## 成员 2：移动步态与 Go2 控制

职责：让 Go2 安全执行前进、转向、跳障、避障、台阶、停止；异常时必须能立刻停。

主负责文件：

- `src/rk_locomotion/rk_locomotion/gait_control_node.py`
- `src/rk_locomotion/rk_locomotion/gait_basic_test_node.py`
- `src/rk_locomotion/config/gait_params.yaml`
- `src/rk_locomotion/launch/gait_control.launch.py`
- `src/rk_unitree_driver/rk_unitree_driver/cmd_vel_bridge_node.py`
- `src/rk_unitree_driver/rk_unitree_driver/go2_motion_client.py`
- `src/rk_unitree_driver/rk_unitree_driver/safety_monitor.py`
- `src/rk_unitree_driver/config/go2_driver.yaml`
- `src/rk_go2_sdk_bridge/scripts/cmd_vel_udp_forwarder.py`
- `src/rk_go2_sdk_bridge/launch/go2_sdk_udp_bridge.launch.py`
- `src/rk_tools/rk_tools/two_step_walk_test_node.py`

优先任务：

| 优先级 | 任务 | 验收标准 |
| --- | --- | --- |
| P0 | 确认 `/navigation/cmd_vel` 到 Go2 桥可用 | 低速前进、原地转、停止均成功 |
| P0 | 确认 watchdog、限速、停机保护有效 | 超时自动停，超速被拒绝或限幅 |
| P1 | 封装起点和终点跳障动作 | 单独测试可重复成功 |
| P1 | 封装避障区低速动作序列 | 尽量不碰挡板通过 |
| P1 | 封装台阶上下动作 | 先稳定拿基础分，再尝试高台阶 |
| P2 | 提供平台前微调动作 | 状态机可调用低速前后和转向动作 |

成员 2 对外只承诺：

- 输入 `/navigation/cmd_vel`：来自成员 1 的巡线速度。
- 输入 `/gait/command_json`：固定动作或调试动作命令。
- 输出 `/gait/control_lock`：固定动作执行期间置 true，让导航节点暂停发布 cmd_vel。
- 提供 `/locomotion/execute_motion`：给成员 3 调用固定动作。

动作名建议统一：

| 动作名 | 用途 |
| --- | --- |
| `start_jump` | 起点障碍 |
| `avoid_zone` 或 `pass_obstacle` | 避障区 |
| `stairs_up_down` | 台阶上下 |
| `finish_jump` | 终点障碍 |
| `final_stop` | 最终停止 |
| `turn_left_90` / `turn_right_90` | 固定转向 |
| `low_speed_adjust` | 平台前低速微调 |

建议自测命令：

```bash
python3 -m py_compile src/rk_locomotion/rk_locomotion/gait_control_node.py
python3 -m py_compile src/rk_unitree_driver/rk_unitree_driver/cmd_vel_bridge_node.py
ros2 launch rk_unitree_driver go2_cmd_vel_bridge.launch.py backend:=mock
ros2 run rk_tools two_step_walk_test_node
ros2 launch rk_locomotion gait_control.launch.py
ros2 topic echo /gait/control_lock
```

## 成员 3：任务状态机、机械臂与识别

职责：把巡线、固定运动、机械臂抓放、标志识别串成完整比赛流程。

主负责文件：

- `src/rk_mission/rk_mission/mission_state_machine_node.py`
- `src/rk_config/config/mission/competition.yaml`
- `src/rk_config/config/arm/d1_presets.yaml`
- `src/rk_interfaces/action/ExecuteMotion.action`
- `src/rk_interfaces/action/ExecuteArmTask.action`
- `src/rk_interfaces/action/RunMission.action`
- `src/rk_interfaces/msg/SignDetection.msg`
- `src/rk_interfaces/msg/SignDetectionArray.msg`
- `src/rk_interfaces/msg/ItemTag.msg`
- `src/rk_interfaces/msg/ItemTagArray.msg`
- `src/rk_perception/rk_perception/mock_sign_detector_node.py`
- `src/rk_perception/rk_perception/mock_item_tag_node.py`
- `src/rk_tools/rk_tools/mock_arm_server.py`
- `src/rk_tools/rk_tools/mock_locomotion_server.py`
- `src/rk_tools/rk_tools/mission_client_node.py`

优先任务：

| 优先级 | 任务 | 验收标准 |
| --- | --- | --- |
| P0 | mock 全流程从 START 跑到 DONE | `mock_competition.launch.py` 不无限等待 |
| P0 | 每个状态设置 timeout 和失败处理 | 失败进入 stop 或明确跳过 |
| P1 | 机械臂抓放动作预设 | `d1_presets.yaml` 有可实测姿态 |
| P1 | 标志识别结果到动作映射 | 放置平台、警示动作选择正确 |
| P2 | 支持分段执行 | 可指定起止 stage 做实机联调 |

成员 3 对外只承诺：

- 发布 `/mission/start` 和 `/mission/stop` 控制成员 1。
- 调用 `/locomotion/execute_motion` 控制成员 2。
- 调用 `/arm/execute_task` 控制机械臂。
- 读取 `/perception/sign_detections` 和 `/perception/item_tags`。

建议状态流程：

```text
PRECHECK
-> WAIT_START
-> START_JUMP
-> FOLLOW_TO_AVOID_ENTRY
-> AVOID_ZONE
-> FOLLOW_TO_STAIRS
-> STAIRS_UP_DOWN
-> FOLLOW_TO_PICK_PLATFORM
-> DETECT_PICK_SIGN
-> PICK_START_ITEM
-> FOLLOW_TO_TRANSFER_PLATFORM
-> DROP_START_ITEM
-> PICK_FIELD_ITEM
-> FOLLOW_TO_CHECK_POINT
-> DETECT_WARNING_SIGN
-> DO_WARNING_ACTION
-> FOLLOW_TO_PLACE_PLATFORM
-> PLACE_FIELD_ITEM
-> FOLLOW_TO_FINISH_JUMP
-> FINISH_JUMP
-> RETURN_TO_START_ZONE
-> FINAL_STOP
-> DONE
```

建议自测命令：

```bash
python3 -m py_compile src/rk_mission/rk_mission/mission_state_machine_node.py
ros2 launch rk_bringup mock_competition.launch.py auto_start:=false
ros2 run rk_tools mission_client_node
```

## 公共接口冻结表

| 类型 | 名称 | 负责人 | 备注 |
| --- | --- | --- | --- |
| Topic | `/perception/line_track` | 成员 1 | `rk_interfaces/msg/LineTrack` |
| Topic | `/navigation/cmd_vel` | 成员 1/2 | 成员 1 导航发布，成员 2 桥接/动作层使用 |
| Topic | `/gait/control_lock` | 成员 2 | true 时成员 1 暂停 cmd_vel 输出 |
| Topic | `/mission/start` | 成员 3 | 启动巡线 |
| Topic | `/mission/stop` | 成员 3 | 停止巡线 |
| Action | `/mission/run` | 成员 3 | 完整比赛任务入口 |
| Action | `/locomotion/execute_motion` | 成员 2/3 | 成员 3 调用，成员 2 实现 |
| Action | `/arm/execute_task` | 成员 3 | 机械臂任务 |

未经三人确认，不改上表名称和消息字段。

## 分支与合并规则

| 成员 | 分支名 | 合并前检查 |
| --- | --- | --- |
| 成员 1 | `feature/vision-nav` | 巡线和导航单测通过，无 conflict 标记 |
| 成员 2 | `feature/locomotion-go2` | Go2 桥、stop、限速、固定动作单测通过 |
| 成员 3 | `feature/mission-arm` | mock 全流程或分段流程可运行 |

合并前统一执行：

```bash
rg -n "<<<<<<<|=======|>>>>>>>" src
colcon build --symlink-install
```

按改动范围追加检查：

```bash
python3 -m py_compile src/rk_navigation/rk_navigation/line_follower_node.py
python3 -m py_compile src/rk_perception/rk_perception/real_line_tracker_node.py
python3 -m py_compile src/rk_locomotion/rk_locomotion/gait_control_node.py
python3 -m py_compile src/rk_mission/rk_mission/mission_state_machine_node.py
PYTHONPATH=src/rk_perception:$PYTHONPATH python3 -m pytest src/rk_perception/test/test_real_line_tracker_node.py
```

## 联调顺序

1. 成员 1 单独跑视觉导航：确认图像、`LineTrack`、`cmd_vel`。
2. 成员 2 单独跑 Go2 桥和动作：确认低速、转向、stop、watchdog。
3. 成员 3 单独跑 mock 状态机：确认 stage、timeout、action 调用。
4. 成员 1 + 成员 2 联调：真实 `/navigation/cmd_vel` 到 Go2，先低速。
5. 成员 2 + 成员 3 联调：固定动作和状态机切换，确认 `/gait/control_lock`。
6. 三人全流程：先分段，再整圈；先保守参数，再提高得分。

## 当前阻塞清单

| 问题 | 负责人 | 状态 |
| --- | --- | --- |
| `line_follower_node.py` 曾有 merge conflict 残留 | 成员 1 | 已整理为明确的 `/gait/control_lock` 订阅接口 |
| `rk_config/config/arm/d1_presets.yaml` 为空 | 成员 3 | 需补机械臂实测预设 |
| Unitree 高级动作仍需实机验证 | 成员 2 | 需建立动作库和视频记录 |
| 旧分析文档部分描述已落后于当前代码 | 全员 | 以本文档和当前源码为准 |

