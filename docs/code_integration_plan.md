# 后续代码集成计划

本文档只规划后续代码改造，不执行代码改造。本轮不得修改核心 Python/YAML 实现代码，不得修改现有 topic/action/msg 字段。

## 冻结接口

| 类型 | 名称 | 规划用途 |
| --- | --- | --- |
| Topic | `/perception/line_track` | 巡线结果，导航消费。 |
| Topic | `/navigation/cmd_vel` | 导航或动作层输出给 Unitree bridge 的速度。 |
| Topic | `/gait/control_lock` | 固定动作占用控制权时，导航暂停发布 cmd_vel。 |
| Topic | `/mission/start` | 状态机启动巡线。 |
| Topic | `/mission/stop` | 状态机停止巡线。 |
| Action | `/mission/run` | 完整比赛任务入口。 |
| Action | `/locomotion/execute_motion` | 状态机调用固定运动动作。 |
| Action | `/arm/execute_task` | 状态机调用机械臂任务。 |

## 成员 1：视觉触发和巡线到点

后续改造范围：

- `src/rk_perception/rk_perception/real_line_tracker_node.py`
- `src/rk_navigation/rk_navigation/line_follower_node.py`
- `src/rk_perception/config/perception.yaml`
- `src/rk_navigation/config/navigation.yaml`
- `src/rk_bringup/config/line_nav_params.yaml`

规划内容：

| 能力 | 后续实现方式 | 输出/验收 |
| --- | --- | --- |
| 视觉触发事件 | 使用规则允许对象：抓取平台 1/2 标志、警示标志、物资标签、台阶允许 ArucoTag、平台/障碍/台阶外观、黑线。暂不新增自贴路线标志。 | 每个关键区域有可复现实测触发条件。 |
| 巡线到点策略 | 状态机启动巡线后，导航持续输出 `/navigation/cmd_vel`；到障碍、台阶、平台、检测点、终点前由视觉触发或黑线几何触发停车。 | 入口/平台前停车误差可接受，停车后不继续输出非零速度。 |
| 丢线处理 | 保留短时丢线、转向搜索、搜索超时 stop；关键区降低速度和角速度上限。 | 短时丢线不乱冲，长时丢线自动 stop。 |
| 搜索重获 | 使用已有 `SEARCH_LINE` 和连续稳定帧计数；出口恢复巡线要求连续 `line_visible=true` 且横向误差收敛。 | 避障区出口、台阶后、跳障后能恢复巡线。 |
| 停靠验收 | 启停区采用蓝色区域外观和黑线终点位置作为停车触发；状态机发布 `/mission/stop` 后保持零速度。 | 四足端完全在启停区蓝色区域内。 |

巡线到点阶段建议先覆盖：

```text
LINE_TO_OBSTACLE_ENTRY
LINE_TO_STAIRS
LINE_TO_PICK_PLATFORM
LINE_TO_TRANSFER_PLATFORM
LINE_TO_CHECK_POINT
LINE_TO_PLACE_PLATFORM
LINE_TO_FINISH_JUMP
RETURN_TO_START_ZONE
```

## 成员 2：步态 action server 和避障动作库

后续改造范围：

- `src/rk_locomotion/rk_locomotion/gait_control_node.py`
- `src/rk_locomotion/config/gait_params.yaml`
- `src/rk_unitree_driver/rk_unitree_driver/cmd_vel_bridge_node.py`
- `src/rk_unitree_driver/rk_unitree_driver/safety_monitor.py`
- `src/rk_go2_sdk_bridge/scripts/cmd_vel_udp_forwarder.py`

规划内容：

| 能力 | 后续实现方式 | 输出/验收 |
| --- | --- | --- |
| `/locomotion/execute_motion` action server | 在 `gait_control_node.py` 增加 `ExecuteMotion` action server，action 名固定为 `/locomotion/execute_motion`。 | `mission_state_machine_node.py` 可在真实模式直接调用成员 2 动作。 |
| 复用现有 JSON 动作逻辑 | action callback 将 `motion_name` 映射为当前 `handle_command(fields)` 可执行的字段，不另写一套运动执行逻辑。 | `/gait/command_json` 和 action 调用行为一致。 |
| 保持 `/gait/control_lock` | 所有非 STOP 固定动作执行期间发布 true，结束或失败发布 false；STOP 可中断当前动作。 | 成员 1 导航节点不会和固定动作抢 `/navigation/cmd_vel`。 |
| stop/watchdog | 保留 cmd_vel bridge 的限速、超时 stop、shutdown repeated stop；动作层失败必须先发零速度。 | 超时、异常、用户 stop 都能让 Go2 停止。 |

固定动作名规划：

| 动作名 | 规划用途 | 推荐底层组合 |
| --- | --- | --- |
| `ENTER_OBSTACLE_ZONE` | 低速进入避障区 | `LOW_SPEED_MOVE`，短时低速直行。 |
| `OBSTACLE_FORWARD_SLOW` | 通道内慢速前进 | `LOW_SPEED_MOVE`，更低 `vx`。 |
| `OBSTACLE_TURN_LEFT` | 避障区左转修正 | `TURN_IN_PLACE` 或低速小角速度。 |
| `OBSTACLE_TURN_RIGHT` | 避障区右转修正 | `TURN_IN_PLACE` 或低速小角速度。 |
| `OBSTACLE_SIDE_ADJUST` | 侧向/姿态微调 | 若底层不支持 `vy`，先用前进+转向组合替代。 |
| `EXIT_OBSTACLE_ZONE` | 低速离开避障区 | `LOW_SPEED_MOVE`，出口 stop。 |
| `OBSTACLE_STOP` | 避障异常保护 | `STOP`，重复发布零速度。 |

其他动作名保持与状态机一致：

```text
start_jump
avoid_zone
stairs_up_down
stretch
wave
blink_front_light_3
finish_jump
final_stop
```

## 成员 3：状态机、机械臂预设和识别映射

后续改造范围：

- `src/rk_mission/rk_mission/mission_state_machine_node.py`
- `src/rk_config/config/mission/competition.yaml`
- `src/rk_config/config/arm/d1_presets.yaml`
- `src/rk_tools/rk_tools/mock_arm_server.py`
- `src/rk_tools/rk_tools/mock_locomotion_server.py`
- `src/rk_tools/rk_tools/mission_client_node.py`

### `d1_presets.yaml` 预设格式规划

真实机械臂 SDK 接入是后续外部依赖；本阶段先用统一预设格式支撑 mock 和任务状态机。

```yaml
arm_presets:
  ARM_HOME:
    description: arm safe home pose
    steps:
      - name: home
        joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        duration_sec: 1.0
  ARM_SAFE:
    description: safe folded pose before locomotion
    steps:
      - name: safe
        joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        duration_sec: 1.0
  ARM_PICK_START_ITEM:
    description: pick start item from pick platform
    steps:
      - name: approach
        joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        gripper: open
        duration_sec: 1.0
      - name: grasp
        gripper: close
        duration_sec: 0.5
  ARM_PLACE_TRANSFER:
    description: place start item on transfer platform
    steps: []
  ARM_PICK_FIELD_ITEM:
    description: pick field item from transfer platform
    steps: []
  ARM_PLACE_PLATFORM_1:
    description: place field item on platform 1
    steps: []
  ARM_PLACE_PLATFORM_2:
    description: place field item on platform 2
    steps: []
```

预设键名固定：

```text
ARM_HOME
ARM_SAFE
ARM_PICK_START_ITEM
ARM_PLACE_TRANSFER
ARM_PICK_FIELD_ITEM
ARM_PLACE_PLATFORM_1
ARM_PLACE_PLATFORM_2
```

### `mock_arm_server` 读取预设规划

- 启动时读取 `rk_config/config/arm/d1_presets.yaml`。
- `ExecuteArmTask.Goal.task_name` 和 `target` 映射到预设键。
- 找到预设则按 steps 逐步发布 feedback；找不到预设则返回 `success=false` 和明确 message。
- mock 只模拟时序和反馈，不驱动真实机械臂。

### mission 分段执行规划

- 保留现有 `start_stage` 和 `end_stage` 参数。
- 每个阶段进入时记录 stage、开始时间、输入条件。
- 每个阶段退出时记录成功/失败、耗时、下一阶段。
- 任一 action 超时进入失败路径：先 `/mission/stop`，再调用 stop/final_stop 或进入 ERROR_STOP 记录。
- 分段测试使用：

```bash
ros2 launch rk_bringup mock_competition.launch.py auto_start:=false
ros2 param set /mission_state_machine_node start_stage FOLLOW_TO_PICK_PLATFORM
ros2 param set /mission_state_machine_node end_stage PICK_START_ITEM
ros2 run rk_tools mission_client_node
```

### 视觉触发接入规划

| 触发源 | 来源 | 状态机用途 |
| --- | --- | --- |
| 抓取平台 1/2 识别标志 | `/perception/sign_detections` | 选择 `place_platform_1` 或 `place_platform_2`。 |
| 检测平台警示标志 | `/perception/sign_detections` | 映射警示动作。 |
| 物资标签 | `/perception/item_tags` | 抓取前确认 `start_item` 或 `field_item`。 |
| 台阶允许 ArucoTag | 后续视觉检测节点或同一感知输出 | 台阶前停车辅助。 |
| 平台/障碍/台阶外观 | 后续视觉检测节点或调参记录 | 到点停车辅助。 |
| 黑色导引线 | `/perception/line_track` | 巡线、恢复巡线、丢线处理。 |

### 识别映射规划

| 识别结果 | 状态机目标 |
| --- | --- |
| `1`, `one`, `place_1`, `platform_1`, `place_platform_1`, `marker_1` | 一号放置平台，`ARM_PLACE_PLATFORM_1` |
| `2`, `two`, `place_2`, `platform_2`, `place_platform_2`, `marker_2` | 二号放置平台，`ARM_PLACE_PLATFORM_2` |
| 当心触电 / `electric_shock` | `stretch` |
| 当心强氧化物 / `strong_oxidizer` | `wave` |
| 当心辐射 / `radiation` | `blink_front_light_3` |

## 完整真实 launch 规划

规划文件名：`src/rk_bringup/launch/competition_full.launch.py`。本轮只写规划，不实现。

该 launch 后续应包含：

| 节点 | 包 | 用途 | 参数来源 |
| --- | --- | --- | --- |
| RealSense camera | `realsense2_camera` | RGB 图像输入 | 相机参数或 launch argument |
| `real_line_tracker_node` | `rk_perception` | 真实黑线检测 | `line_nav_params.yaml` |
| `line_follower_node` | `rk_navigation` | 巡线控制 | `line_nav_params.yaml` |
| `gait_control_node` | `rk_locomotion` | 固定动作和 action server | `gait_params.yaml` |
| `cmd_vel_udp_forwarder.py` | `rk_go2_sdk_bridge` | `/navigation/cmd_vel` 到 Go2 SDK UDP server | launch 参数 |
| `mission_state_machine_node` | `rk_mission` | 完整任务状态机 | `competition.yaml` |
| arm mock/adapter | `rk_tools` 或后续机械臂包 | `/arm/execute_task` | `d1_presets.yaml` |

launch 参数规划：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `debug` | `false` | 打开视觉和导航 debug。 |
| `bridge_type` | `sdk_udp` | 默认使用巡线已验证的 Go2 SDK UDP 桥接。 |
| `start_realsense` | `true` | 是否由 launch 启动相机。 |
| `use_mock_arm` | `true` | 机械臂真实 SDK 未接入前使用 mock。 |
| `auto_start` | `false` | 比赛前默认不自动开始，等待裁判信号。 |
| `image_topic` | `/camera/camera/color/image_raw` | 真实巡线输入图像。 |

## 风险和优先级

| 优先级 | 风险 | 负责人 | 处理方式 |
| --- | --- | --- | --- |
| P0 | mission 调用真实 locomotion action 时无 server | 成员 2 | 在 `gait_control_node.py` 规划并后续实现 action server。 |
| P0 | 巡线和固定动作同时发 `/navigation/cmd_vel` | 成员 1/2/3 | 使用 `/gait/control_lock` 和 `/mission/stop` 串联。 |
| P0 | merge conflict 标记进入源码 | 全员 | 每次合并前执行 `rg` 检查。 |
| P1 | 机械臂预设为空 | 成员 3 | 先 mock 预设格式，再实机标定。 |
| P1 | 视觉触发误判 | 成员 1/3 | 只用规则允许对象，关键阶段加连续帧稳定条件。 |
| P1 | 避障区碰板 | 成员 2 | 降速、分动作、每步可 stop。 |
| P2 | `rk_common` 空骨架 | 全员 | 暂不抽公共逻辑，避免过早耦合。 |

## 必写测试命令

```bash
rg -n "<<<<<<<|=======|>>>>>>>" src
python3 -m py_compile src/rk_navigation/rk_navigation/line_follower_node.py
python3 -m py_compile src/rk_mission/rk_mission/mission_state_machine_node.py
python3 -m py_compile src/rk_locomotion/rk_locomotion/gait_control_node.py
PYTHONPATH=src/rk_perception:$PYTHONPATH python3 -m pytest src/rk_perception/test/test_real_line_tracker_node.py
colcon build --symlink-install
```
