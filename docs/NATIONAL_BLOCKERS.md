# 国赛基线阻塞项

## 判定原则

本文依据 `codex/national-control-architecture` 分支、基线提交 `ded2631` 之上的
仓库源码静态审计及本轮 VM 基础检查。没有机器人端日志、视频、SDK ACK 或
可追溯测试记录的功能，一律不标为硬件已验证。

严重度：

- `P0`：会让关键评分动作必然失败、误报成功，或形成失控风险。
- `P1`：可能走错评分分支、破坏完整流程或导致现场不可复现。
- `P2`：基础质量、可维护性或证据链缺口。

## 1. 用户指定的七项核查

### 1.1 `stairs_up_down` 是否有真实 `ExecuteMotion` 实现

结论：**没有，P0。**

- 完整状态机确实发送 `motion_name=stairs_up_down`：`src/rk_mission/rk_mission/mission_state_machine_node.py:439-441`。
- 真实 `GaitControlNode` ActionServer 对未映射名称返回 `unsupported motion_name` 并 abort：`src/rk_locomotion/rk_locomotion/gait_control_node.py:676-687`。
- 完整映射表 `gait_control_node.py:725-770` 没有 `stairs_up_down`。
- mock locomotion 会对任意名称固定等待后返回 success：`src/rk_tools/rk_tools/mock_locomotion_server.py:26-49`。这不能算楼梯实现。

影响：台阶最高 30 分当前没有真实执行入口；mock 全流程会掩盖该失败。

验收门槛：真实 gait server 接受该 goal；动作有明确上下台阶、失败 stop、足端/姿态完成条件；在标准台阶连续硬件通过并保存 action result 与视频。

### 1.2 `start_jump` 和 `finish_jump` 是否真正调用跳跃 SDK

结论：**完整状态机路径没有调用跳跃 SDK，P0。**

- 名称只映射到 `JUMP_START_OBSTACLE` / `JUMP_END_OBSTACLE`：`gait_control_node.py:732-736,838-841`。
- `execute_jump_obstacle()` 只执行两段定时前进 Twist、停顿和等待：`gait_control_node.py:1274-1346`。
- 准备姿态明确记录为未接线 TODO：`gait_control_node.py:1325-1330`。
- 真正调用 `SportClient.FrontJump()` 的代码只在独立 C++ 工具 `src/rk_go2_sdk_bridge/src/go2_sdk_motion_action.cpp:83-85`。
- standalone `obstacle_direct_route_node` 有一次独立 `front_jump` 调试路径，但它没有接入完整 mission 的 `start_jump/finish_jump` action 闭环。

影响：两段低速前进可能被日志标成 jump completed，但不会执行真正跳跃；终点障碍没有可复用的独立 SDK 闭环。

验收门槛：完整 action 路径实际调用 SDK、检查返回码、支持 cancel/timeout，并有落地稳定判断和起终点分别验证的记录。

### 1.3 New arm SDK bridge 是否有真实订阅者和硬件 ACK

结论：**两者都没有，P0。**

- `SdkBridgeArmAdapter` 只创建 `/arm/sdk_bridge/command_json` publisher：`src/rk_arm_control/rk_arm_control/adapters/sdk_bridge_adapter.py:31-45`。
- move/gripper 发布 JSON 后只按 `duration_sec` sleep，再返回成功：`sdk_bridge_adapter.py:55-82,140-155`。
- 全仓没有该 topic 的 subscriber，也没有 ACK topic/service/action。
- `new_arm_task_node` 依据 adapter 返回的布尔值把 step/task 标为成功，因此“无人接收”也可能获得成功结果。
- `new_arm_poses.yaml:36-43` 明确说明当前关节角是占位值。

影响：抓、放、夹爪或 stop 都可能未到达硬件，却被完整 mission 当成成功；涉及 70 分机械臂评分项。

验收门槛：仓内或明确外部版本化 vendor bridge；启动时 subscriber readiness；每条命令携带 sequence；硬件 ACK/错误码/超时；action result 只由匹配 ACK 决定；点位完成空载和负载标定。

### 1.4 `white_bar_action_done` 由谁发布

结论：**仓库内无人发布，P0。**

- `LineCourseMissionNode` 只订阅 `/mission/white_bar_action_done`：`mission_state_machine_node.py:910-915,1157-1159`。
- 收到 `true` 才进入 `REACQUIRE_LINE`：`mission_state_machine_node.py:1394-1402`。
- 未收到时默认 5 秒后进入 `EMERGENCY_STOP`：`mission_state_machine_node.py:1410-1415`。
- `line_nav_params.yaml:244-249` 也写明白横线动作尚未配置。

影响：真实巡线遇到白横线后只能依赖仓外人工发布或必然超时停车。

验收门槛：明确唯一动作执行者、触发条件、动作 result 到 Bool 的转换、失败状态和重复消息语义；加入 launch 与集成测试。

### 1.5 所有 `follow_to_*` 的结束条件

结论：**固定时间结束，不是到达确认，P0。**

完整状态机使用：

- `follow_to_avoid_entry`
- `follow_to_stairs`
- `follow_to_pick_platform`
- `follow_to_transfer_platform`
- `follow_to_check_point`
- `follow_to_place_platform_1` 或 `follow_to_place_platform_2`
- `follow_to_finish_jump`
- `return_to_start_zone`

证据：

- `mission_state_machine_node.py:427-537` 逐项调用 navigation segment。
- gait 把所有 `follow_to_*` 和 `return_to_*` 统一映射为 `WAIT_NAVIGATION_SEGMENT`：`gait_control_node.py:762-768`。
- `execute_wait_navigation_segment()` 只等 duration 到期就 success：`gait_control_node.py:1048-1086`。
- 默认时长 3 秒：`src/rk_locomotion/config/gait_params.yaml:23`。

没有里程计、视觉地标、平台/障碍到达、蓝区进入或 line-course state 的 result 条件。

影响：场地尺寸、速度或起点偏差会使状态机在错误位置切换跳跃、机械臂或警示动作。

验收门槛：每个 segment 单独定义可观测到达条件、超时失败、停止确认和恢复策略；不得继续使用同一个固定等待冒充全部路线到达。

### 1.6 默认放置平台和默认警示动作是否可能造成错误得分

结论：**会，P0。**

- 配置默认 `allow_detection_fallback=true`、`default_place_target=place_platform_1`、`default_warning_action=stretch`：`src/rk_config/config/mission/competition.yaml:21-26`。
- 识别超时后状态机确实采用默认值：`mission_state_machine_node.py:696-725`。
- 若真实目标是平台 2 或警示牌不是 stretch，会主动执行错误评分动作。
- new arm 又把平台 1、2 都映射到同一 `PLACE_TARGET`，且只有一套放置姿态：`new_arm_task_node.py:626-635`、`new_arm_poses.yaml:83-93,161-176`。
- 即使警示识别正确，真实 gait 映射也不支持 `stretch/wave/blink_front_light_3`。

影响：识别故障不会安全失败，而可能造成错误放置/错误动作；日志仍可能表现为已选择有效 target。

验收门槛：正式模式关闭默认评分动作 fallback；未知/低置信结果进入明确 stop/retry；平台 1/2 必须有独立路线到达和独立已标定 arm target；三类警示动作逐一真实验证。

### 1.7 是否存在多个节点同时发布最终 `cmd_vel`

结论：**真实巡线 launch 已建立唯一发布者基础；完整比赛和误启 standalone 的
风险仍未完成硬件验证，P1。**

当前 `competition_line_nav.launch.py` 和 `start_line_system.sh` 的真实数据流是：

```text
line_follower suggested
-> line_course_mission
-> /control/mission_cmd
-> command_mux
-> /navigation/cmd_vel
-> UDP forwarder
```

该组合中 `command_mux_node` 是唯一预期的最终 publisher；启动/检查脚本会验证
`Publisher count: 1`。line-course 默认输出已改为 `/control/mission_cmd`，
gait 默认输出已改为 `/control/locomotion_cmd`。

正常停机也保持该所有权：`stop_line_system.sh` 通过 mux 的
`/safety/estop` `SetBool` service 置位急停，要求返回成功并观测到 mux 发布零速
后才终止节点。只有 service 不存在、类型不符或调用失败时，
`EMERGENCY FALLBACK` 才直发一次零 Twist；该异常路径会绕过正常唯一发布者
架构，不能计为所有权验收通过。

以下 standalone/tool 路径仍显式保留直接发布最终 topic：

- mock/vision debug launch 中的 `line_course_mission_node`
- obstacle/sign debug launch 中的 `gait_control_node`
- `obstacle_direct_route_node`
- `keyboard_route_node`
- `cmd_vel_speed_sweep_node`
- `two_step_walk_test_node`
- `stop_line_system.sh` 的 `EMERGENCY FALLBACK` 一次性零 Twist CLI publisher
  （仅 service 缺失/失败时）

`line_follower_node` 默认发布 suggested topic；`/control/line_cmd` 当前只是 mux
预留输入。任何参数/remap 仍可能绕过上述契约。

影响：真实巡线的静态控制权冲突已收敛，但同时启动任一 standalone/helper
仍会出现多个 DDS publisher；完整 mission/gait/arm 又尚未进入真实比赛 launch，
所以不能推断全赛程接管已经验证。

验收门槛：机器人 Foxy 上启动正式组合后，
`ros2 topic info -v /navigation/cmd_vel` 必须显示唯一
`command_mux_node` publisher；测试/helper 不得存在；分别验证 gait lock、
arm lock、estop、超时和解锁后新命令恢复。

## 2. 其他关键阻塞项

### P0

1. **没有真实完整比赛 launch。** `mock_competition.launch.py` 组合完整 stage 但全部关键硬件为 mock；`competition_line_nav.launch.py` 只组合真实巡线子系统，缺完整 mission、gait action、arm、real sign/item 和 white-bar executor。
2. **警示动作完整路径不可执行。** 完整 mission 把三种动作发送给 gait，但 gait 不支持；独立 `sign_action_executor_node` 的默认 step 同时包含 `wait_sec` 和 `sdk_action`，执行器先处理 wait 并 `continue`，可能根本不进入 SDK 分支（`sign_action_executor_node.py:269-302`）。
3. **没有真实 item-tag/object-XY producer。** 起始/场地物资只在 mock item node 中出现；完整 mission 还忽略 item-tag 等待失败并继续调用 arm。
4. **完整 mission 的 `WAIT_START` 不是真正起跑边界。** 它只是默认 0.2 秒 sleep；且 `auto_start=true` 可在节点启动后自动发 goal。

### P1

5. **两个状态机职责重叠但没有正式组合契约。** 完整 mission 控制 `/mission/start/stop`，line-course 独立处理红圈、白横线、角点和蓝区；评分路线和 line-course 事件如何对应没有 action/result 接口。
6. **Go2 UDP 路径无 ACK。** mux status 和停机脚本的 ROS 零速抽样只记录软件链；forwarder 仍会第二次限幅并只发送 UDP，外部 `go2_sdk_udp_server` 不在仓库，无法从本仓库证明最终 payload、解析、SDK 执行、实体停止或硬件状态。
7. **SDK direct tools 是条件构建。** `unitree_sdk2` 未找到时 CMake 只 warning，不生成 `go2_sdk_motion_action` 等可执行文件；多个 launch 又依赖这些工具或机器人绝对路径。
8. **多个替代 bridge/arm/locomotion server 可争抢同一接口。** setup 和 launch 没有统一互斥选择机制，错误组合时 action server 或机器人后端不唯一。
9. **全仓 pytest 仍不是单一全绿入口。** 裸 `pytest` 当前不在 PATH，返回 127；原始根级 `python3 -m pytest` 的 22 个 collection errors 已分类为 18 个同名测试模块冲突、3 个 `PYTHONPATH` 问题、1 个 D1 动态库问题，原收集阶段没有断言失败。D1 测试在 import 阶段可能直接驱动机械臂，不能自动收集。最新 `rk_safety` 定向结果为 37 passed；安全 `src/` 收集共 74 项，逐包为 65 passed、9 skipped、0 failed；干净 colcon 为 54 tests、0 errors、0 failures、1 skipped。统一安全入口和正确命令见 `docs/TEST_BASELINE.md`。

### P2

10. **三个配置 YAML 为空。** `rk_config/config/arm/d1_presets.yaml`、`camera/d435i.yaml`、`robot_profiles/go2.yaml` 可语法解析但不提供配置内容。
11. **package 元数据仍称多个混合包为 mock。** 不影响运行，但会误导启动与验收判断。
12. **VM 测试结果不能替代机器人端硬件验证。** 当前只有源码层面的 Foxy/Humble 静态兼容性审计和 VM/Humble 构建测试结果，缺少机器人 Foxy 构建与运行记录。`colcon test` 通过只证明当前环境中已注册测试的结果；不能替代机器人端 topic/action 联调、Unitree SDK/UDP 实际执行、机械臂硬件 ACK、真实相机输入及实物动作验收。

### 已处理的控制权基础

- 真实 `competition_line_nav` 与 `start_line_system.sh` 已将 line-course 候选输出
  接到 `/control/mission_cmd`，再由 mux 独占最终 `/navigation/cmd_vel`。
- mux 已实现 estop、arm lock、gait lock、mission、line 的固定优先级；默认超时
  0.5/0.5/0.3 秒，20 Hz 输出，0.60/0.15/1.30 限值。
- 解锁会清旧候选，时间倒退输出零，NaN/Inf 拒绝，合法超限 clamp，并发布
  JSON 状态。
- mux 同时处理同名 `SetBool` service 与 `Bool` topic，两种入口统一调用 estop
  转换；状态真实变化时清空三个候选的命令、时间戳和有效性，重复置位不反复
  清缓存。
- 正常停机通过 service 置位、mux 零速确认、再停进程；软件闭环已在
  VM/Humble 验证。
- 这些结论只说明控制权软件基础存在。机器人/Foxy 的 UDP/SDK 与实体停止、
  完整真实 launch、跳跃、台阶、机械臂 ACK、白横线、170 分任务均未验收。

### 已处理的仓库卫生项

- **Git 索引生成物已处理。** 原宽泛统计的 5101 个 `build/install/log` 路径中，有 9 个是必须保留的 Unitree SDK `common/log/*.hpp` 源码头文件；实际生成树为 5092 项。缓存/环境命中 54 项，其中 49 项与 install 树重叠，另 5 项属于根 `.venv`。本轮用精确路径从索引移除的唯一文件总数为 5097，本地文件未删除，9 个源码头文件和必要 SDK 库继续追踪。
- `.gitignore` 和 `national_preflight.sh` 已改为精确识别根构建输出及已知第三方生成树，不再把 SDK 源码 `log` 目录误判为日志。
- 这是仓库卫生修复，不代表机械臂 ACK、跳跃、台阶、状态机、感知或 `cmd_vel` 控制权等比赛功能已经完成。

## 3. 风险最高的 10 个问题

| 排名 | 风险 | 直接后果 |
| ---: | --- | --- |
| 1 | 急停只完成 VM 软件闭环，机器人 UDP/SDK 与实体停止未验证 | ROS 零速不能证明 Go2 已收到命令或在安全距离内停止 |
| 2 | new arm 无 subscriber/ACK 却可返回成功 | 70 分机械臂相关流程可能全部假成功 |
| 3 | `stairs_up_down` 真实 server 不支持 | 台阶 30 分必然在 action 层失败 |
| 4 | 起终点 jump 不调用真实跳跃 SDK | 可能低速撞障碍，同时日志显示 completed |
| 5 | 所有 `follow_to_*` 仅固定 3 秒 | 在错误位置切换危险动作或机械臂 |
| 6 | 默认平台 1/default stretch 自动兜底 | 识别失败时主动执行错误评分动作 |
| 7 | `white_bar_action_done` 无发布者 | 白横线流程必然依赖仓外注入或超时急停 |
| 8 | 没有真实完整比赛 launch；standalone 仍可直发最终速度 | 各调试岛无法证明全流程接口、顺序和唯一控制权正确 |
| 9 | 真实物资感知缺失且 tag 失败被忽略 | 机械臂可能在无目标时盲抓 |
| 10 | 缺少机器人 Foxy 端到端/硬件证据 | VM 静态或构建成功可能被误当成国赛可用 |

## 4. 下一轮进入实现前的最小退出条件

1. **已完成：** 精确处理 Git 索引中的 5092 个生成物和 5 个根 `.venv` 条目；保留并记录 9 个 SDK `log` 源码头文件及必要预编译库。
2. **部分完成：** 已建立安全统一测试入口和 22 个收集错误分类，`rk_tools` pep257 失败已修复；仍需决定如何把其他包测试正式注册进 colcon。
3. 受控归档官方赛题规则和评分表，记录版本、哈希与获取日期，并冻结评分矩阵。
4. **控制权基础已完成、硬件验证未完成：** 真实巡线 launch 由 mux 独占最终
   topic，SetBool service 与 Bool topic 已统一接入 mux；下一步先在机器人
   Foxy 验证正常停机、UDP/SDK 与实体停止，再把完整 mission/gait/arm 按同一
   契约纳入真实 launch。任何 `EMERGENCY FALLBACK` 都不得计为正常路径通过。
5. 为 `stairs_up_down`、起终点跳跃和三类警示动作分别确定真实 SDK 调用与 action result 语义。
6. 为 new arm 增加真实 bridge readiness 与 ACK，再标定平台 1/2 独立点位。
7. 给每个 `follow_to_*` 定义可观测到达条件，禁止统一固定等待。
8. 明确白横线动作执行者和 done 发布者。
9. 补真实 item/object producer，并让检测失败阻止盲抓。
10. 在机器人 Foxy 环境验证 mux 唯一发布者、急停、lock、超时、解锁新命令和
    UDP 二次限幅，再按“分项 -> 子链 -> 全链”保存可复现测试证据；不得用 VM
    核心测试、ROS smoke 或 `colcon test` 结果替代硬件验收。
