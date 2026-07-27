# 起点与终点 FrontJump 监督执行和验证计划

## 1. 目的

本文设计第一个实际开发 PR：

```text
feat(locomotion): supervise start and finish FrontJump execution
```

该 PR 的目标不是证明机器人已经物理跨越障碍，而是把当前用两段定时 Twist
冒充跳跃的实现，替换为在现有 ROS2 控制链中受监督调用 SDK2 FrontJump 的执行器。

本文只形成设计。本轮不修改：

- 运行代码或参数；
- `ExecuteMotion.action`；
- 其他消息、Action 或 Service；
- line tracker、line follower、白色障碍 detector 或完整 mission；
- 迷宫、楼梯、机械臂；
- UDP 协议或 legacy。

### 1.1 语义标签

- `[CURRENT]`：当前仓库中已存在的接口、映射和运行行为。
- `[PROPOSED]`：首个 FrontJump PR 或后续集成的设计，当前尚未实现。
- `[NEEDS_FIELD_EVIDENCE]`：起点/终点独立数值、姿态阈值和物理结论，必须实机验证。

## 2. [CURRENT] 当前仓库事实

### 2.1 真实 Action 接口

`src/rk_interfaces/action/ExecuteMotion.action` 的完整 schema 为：

```text
string motion_name
---
bool success
string message
---
string current_step
float32 progress
```

因此现有接口只有：

- goal：`motion_name`
- result：`success`、`message`
- feedback：`current_step`、`progress`

现有 schema 没有：

- `stage`
- `failure_code`
- `sdk_command_accepted`
- `post_settle_completed`
- `physical_verified`
- `physical_crossing_unverified`
- `stability_verified`

首个 PR 禁止假设或新增这些 typed 字段。
`sdk_command_accepted`、`post_settle_completed` 和
`physical_crossing_unverified` 只允许写入结构化日志或现有 result `message`
文本；完整 mission 后续可派生为 `[PROPOSED]` RunMission 内部记录。

### 2.2 当前 start_jump 和 finish_jump 入口

`gait_control_node.py` 当前映射：

```text
start_jump  -> JUMP_START_OBSTACLE -> execute_jump_obstacle("start")
finish_jump -> JUMP_END_OBSTACLE   -> execute_jump_obstacle("end")
```

`execute_jump_obstacle()` 当前行为：

1. 预检查；
2. 发布零速并固定等待；
3. `prepare_obstacle_pose()`，但步态/高度适配器仍为 TODO；
4. 以 `jump_phase1_vx` 发布第一段定时 Twist；
5. 固定暂停；
6. 以 `jump_phase2_vx` 发布第二段定时 Twist；
7. 固定恢复等待；
8. 返回 `"<command> completed"`。

当前实现没有调用 `SportClient.FrontJump()`，也没有证明机器人离地、跨越或落地。

### 2.3 当前 Action 执行和取消

- goal 只检查 `motion_name` 非空；
- feedback 只写 `current_step` 和 `[0,1]` 的 `progress`；
- result 只写 `success` 和 `message`；
- cancel callback 设置 `_emergency_stop=True` 并调用 motion stop；
- `handle_command()` 在动作开始时发布 gait lock；
- 当前普通退出在函数尾部释放 gait lock；
- 当前异常和早退路径没有形成 FrontJump 专用的 finally 零速证明。

### 2.4 真实 SDK2 helper

现有可执行程序调用形式：

```text
go2_sdk_motion_action <network_interface> front_jump <wait_sec>
```

`front_jump` 分支直接调用：

```text
SportClient.FrontJump()
```

helper 行为：

- SDK client timeout 设置为 10 秒；
- 在 stdout 打印 `SDK action result: <int32>`；
- SDK 返回值为 0 时进程退出码为 0；
- SDK 返回值非 0 时进程退出码为 1；
- 参数或异常错误也退出 1；
- `wait_sec` 是 SDK 调用返回后的 helper 内等待。

首个 PR 应使用：

```text
go2_sdk_motion_action <network_interface> front_jump 0
```

post-settle 由 ROS executor 独立监督，避免把 helper 内固定等待误当成动作结果。

### 2.5 当前稳定性检查

当前：

```python
def is_robot_stable(self):
    # TODO: Subscribe to Unitree low_state/high_state IMU roll and pitch
    return True
```

它没有读取：

- 新鲜 roll；
- 新鲜 pitch；
- 三轴角速度；
- 真实状态消息时间。

因此当前 `is_robot_stable()` 不是有效的物理稳定证据。首个 PR 不得用它宣称
`stability_verified`，也不得因为它返回 True 宣称机器人成功落地。

### 2.6 command_mux 的相关语义

command_mux：

- gait lock 为 true 时只选择新鲜 locomotion candidate；
- locomotion candidate 不新鲜时输出零；
- gait lock 从 true 释放为 false 时使 line、mission、locomotion 旧缓存全部失效；
- 解锁后必须等待新的 line 或 mission 命令才能恢复运动；
- `/control/cmd_mux_status` 是 JSON 诊断，记录 active source、reason、lock 和 freshness；
- `/navigation/cmd_vel` 是唯一最终软件速度。

首个 PR 必须复用这些语义，不新增绕过 mux 的速度通道。

## 3. [PROPOSED] 首个 PR 精确范围

### 3.1 允许修改

后续 PR 仅允许触及：

- `rk_locomotion/gait_control_node.py` 中的 jump 执行和必要的可测试辅助类；
- `rk_locomotion/config/gait_params.yaml` 中 executor 自有的起点/终点独立参数；
- 新增 `rk_locomotion/test/test_front_jump_executor.py` 或等价定向测试；
- 必要时更新 `rk_locomotion/README.md` 的操作和结果语义。

若实现发现必须修改其他运行包、ROS schema 或 UDP 层，应停止并拆分后续 PR，
不得扩大首个 PR。

### 3.2 明确不修改

- `ExecuteMotion.action`；
- tracker、follower、line-course、完整 mission；
- 白色障碍和蓝区 detector；
- 抓取平台或警示标识识别；
- 迷宫、楼梯、机械臂；
- SDK helper 的 Unitree 协议；
- legacy。

### 3.3 复用接口

- Action：`/locomotion/execute_motion`
- goal：`motion_name=start_jump` 或 `motion_name=finish_jump`
- lock：`/gait/control_lock`
- locomotion candidate：`/control/locomotion_cmd`
- 最终软件速度：`/navigation/cmd_vel`
- 辅助诊断：`/control/cmd_mux_status`
- SDK helper：`go2_sdk_motion_action ... front_jump 0`

## 4. [PROPOSED] 起点和终点独立 profile

起点和终点可共用一个 FRONT_JUMP executor，但 profile 实例必须完全独立。
不得用代码别名或同一可变对象使修改一处参数同步改变另一处。
所有 profile 数值和 stability 阈值均为 `[NEEDS_FIELD_EVIDENCE]`；未完成实机标定前
只能作为安全开发初值。

### 4.1 完整逻辑 profile

| 字段 | `start_jump` | `finish_jump` | 所有者与首个 PR 处理 |
|---|---|---|---|
| `approach_speed` | 独立待标定值 | 独立待标定值 | line-course 所有；首个 PR 只记录契约，不新增未使用 gait 参数 |
| `pre_stop_duration` | 独立待标定值 | 独立待标定值 | executor；取得 lock 并开始发布零后计时 |
| `final_zero_epsilon` | 独立待标定值 | 独立待标定值 | executor |
| `final_zero_confirm_samples` | 独立待标定值 | 独立待标定值 | executor |
| `final_zero_timeout` | 独立待标定值 | 独立待标定值 | executor |
| `sdk_timeout` | 独立待标定值 | 独立待标定值 | executor |
| `post_settle_duration` | 独立待标定值 | 独立待标定值 | executor |
| `stability_thresholds` | 独立待标定值 | 独立待标定值 | 只有接入真实新鲜姿态数据后才启用 |
| `reacquire_profile` | 独立配置名 | 独立配置名 | line-course 所有；后续 PR 接入 |

由于首个 PR 明确不修改 line-course，`approach_speed` 和 `reacquire_profile`
在首个 PR 中只能作为跨模块设计契约，不能假装已生效。首个 PR 只落地 executor
实际消费的参数；后续 line-course PR 才落地另外两个字段。

### 4.2 建议参数命名

executor 自有参数按障碍分开：

```text
front_jump.start.pre_stop_duration
front_jump.start.final_zero_epsilon
front_jump.start.final_zero_confirm_samples
front_jump.start.final_zero_timeout
front_jump.start.sdk_timeout
front_jump.start.post_settle_duration
front_jump.start.stability.roll_max
front_jump.start.stability.pitch_max
front_jump.start.stability.angular_velocity_max
front_jump.start.stability.confirm_samples
front_jump.start.stability.timeout

front_jump.finish.pre_stop_duration
front_jump.finish.final_zero_epsilon
front_jump.finish.final_zero_confirm_samples
front_jump.finish.final_zero_timeout
front_jump.finish.sdk_timeout
front_jump.finish.post_settle_duration
front_jump.finish.stability.roll_max
front_jump.finish.stability.pitch_max
front_jump.finish.stability.angular_velocity_max
front_jump.finish.stability.confirm_samples
front_jump.finish.stability.timeout
```

公共非场景参数：

```text
front_jump.sdk_action_executable
front_jump.sdk_network_interface
front_jump.zero_publish_rate_hz
front_jump.final_cmd_topic
front_jump.cmd_mux_status_topic
```

所有参数必须验证类型、有限性、非负性或正值要求。默认值只能作为安全开发初值，
不能标记为场地验收参数。

## 5. 固定监督顺序

```text
acquire_gait_lock
-> publish_locomotion_zero
-> wait_final_cmd_zero
-> sdk_front_jump
-> post_settle
-> optional_stability_check
-> done
-> finally_keep_zero_and_release_lock
```

不得恢复成：

```text
pre_stop -> acquire_gait_lock -> wait_final_cmd_zero
```

在取得 gait lock 之前，mission 或其他候选源仍可能继续产生非零速度，所以停稳
监督必须从 lock 之后开始。

## 6. 各监督阶段

### 6.1 acquire_gait_lock

进入条件：

- motion_name 已映射为 start 或 finish profile；
- 当前没有其他 gait action；
- estop 未激活；
- goal 尚未取消；
- helper 路径和参数已通过静态预检查。

动作：

1. 记录 `lock_requested_at`；
2. 发布 `/gait/control_lock=true`；
3. 设置本地 `lock_owned=true`；
4. 立即进入 `publish_locomotion_zero`。

不得在此阶段前等待 pre-stop，也不得在此阶段发布非零 Twist。

`cmd_mux_status.gait_lock=true` 可记录为辅助观测，但不是进入下一阶段的唯一协议。
真正的安全门是持续 locomotion 零命令和最终 `/navigation/cmd_vel` 连续归零。

### 6.2 publish_locomotion_zero

取得 lock 后立即向：

```text
/control/locomotion_cmd
```

发布全零 `Twist`。

从此阶段开始直到 finally 释放 lock：

- 以 `zero_publish_rate_hz` 持续发布零；
- 禁止任何 jump executor 路径发布非零 Twist；
- SDK 调用、post-settle、取消、异常和 timeout 期间都保持发布；
- 非有限参数或内部异常也只能退化为零。

`pre_stop_duration` 从第一次 lock 后零命令开始计时。

### 6.3 wait_final_cmd_zero

主要证据是订阅：

```text
/navigation/cmd_vel
```

单条消息满足：

```text
abs(linear.x)  <= final_zero_epsilon
abs(linear.y)  <= final_zero_epsilon
abs(angular.z) <= final_zero_epsilon
```

且三个值都必须有限。

连续确认规则：

- 只统计回调新收到的消息，不能在 timer 中重复统计同一条；
- 记录本地 receive time，消息必须位于当前等待窗口内；
- 非零、非有限、陈旧或缺失使 streak 归零；
- streak 达到 `final_zero_confirm_samples`；
- 从第一次 lock 后零命令起已经经过 `pre_stop_duration`；
- 两个条件都满足后才允许调用 SDK；
- 总等待不得超过 `final_zero_timeout`。

等待期间继续发布 locomotion 零命令。

`/control/cmd_mux_status` 只用于记录：

- `gait_lock`
- `active_source`
- `reason`
- line/mission/locomotion freshness
- final vx/vy/wz

禁止把 JSON status 当作唯一安全控制协议。

证据边界：

- `/navigation/cmd_vel` 连续为零只证明最终软件命令为零；
- cmd_mux status 只证明软件仲裁状态；
- 两者都不能证明机器人物理上完全静止；
- 如果未来需要物理静止门，必须接入真实新鲜姿态/速度数据并单独验收。

### 6.4 sdk_front_jump

启动 helper 时：

- 使用 argv 列表，不使用 shell；
- action 固定为 `front_jump`；
- helper `wait_sec` 固定传 0；
- 显式传入当前 profile 对应的 `sdk_timeout`；
- 捕获 stdout、stderr、进程退出码、启动异常和 elapsed；
- 使用轮询或异步方式，使执行期间仍能处理 cancel、estop 和持续零发布。

在 helper 真正启动前收到 cancel/estop：

- 不启动 helper；
- 返回 canceled/aborted；
- 进入 finally。

helper 启动后：

- 继续发布零；
- cancel 或 estop 可以终止/回收本地 helper 进程；
- 但如果 `FrontJump()` 请求已经发给机器人，终止进程不能撤回物理动作；
- 日志必须保留 `sdk_invocation_started` 和 `sdk_request_may_have_been_sent`。

退出码处理：

| 情况 | 结果 |
|---|---|
| helper 缺失/不可执行 | abort |
| 启动异常 | abort |
| 超过 `sdk_timeout` | 终止并回收进程，abort |
| 退出码非 0 | abort，并记录 stdout/stderr |
| 退出码 0 | 仅记录 `sdk_command_accepted=true`，进入 post-settle |

退出码 0 不得解释为：

- 成功离地；
- 成功跨越障碍；
- 成功落地；
- 实体任务完成。

### 6.5 post_settle

从 helper 以 0 退出后开始：

- 持续发布 locomotion 零命令；
- lock 保持 true；
- 等待当前 profile 的 `post_settle_duration`；
- 持续检查 cancel 和 estop；
- 不发布任何非零 Twist；
- 完成后记录 `post_settle_completed=true`。

固定等待只表示监督流程完成了等待，不是物理落地证明。

### 6.6 optional_stability_check

首个 PR 审计结论是当前没有有效数据源，因此：

- 不得调用恒为 True 的 `is_robot_stable()` 形成成功结论；
- 默认路径记录 `stability_check=unavailable`；
- 继续使用 `physical_crossing_unverified`；
- 不新增虚假的姿态订阅。

只有后续确认 locomotion 能获取真实且新鲜的数据时，才允许单独升级：

- `abs(roll) <= roll_max`
- `abs(pitch) <= pitch_max`
- 角速度范数或各轴均在阈值内
- 消息 age 在上限内
- 连续稳定 N 个样本
- 有独立 `stability_timeout`

即使姿态稳定也只能证明落地后姿态满足阈值，不能单独证明跨越了障碍。实体跨越仍需
赛道位置证据和实机视频。

### 6.7 done

成功进入 done 的最低条件：

- lock 后 locomotion 零持续发布；
- 最终软件速度连续归零；
- helper 退出码 0；
- post-settle 完成；
- 全程没有 cancel、estop、异常或 timeout。

此时：

```text
result.success = true
```

只表示“受监督的 SDK 调用流程完成”。
终点调用成功后，上层 `[PROPOSED]` mission 状态只能命名为
`END_JUMP_SUPERVISED_DONE`，该名称不表示物理越障完成。

建议 `result.message`：

```text
supervised_front_jump_flow_completed;
sdk_command_accepted=true;
post_settle_completed=true;
physical_crossing_unverified=true
```

不得写：

```text
jump completed
physical jump succeeded
obstacle crossed
landing succeeded
```

### 6.8 finally_keep_zero_and_release_lock

所有退出路径，包括成功、失败、取消、estop、异常和 timeout，都必须进入同一
finally：

1. 保持发布 locomotion 零；
2. 再发布一次或一组最终零命令；
3. 清理/回收 helper 进程；
4. 清理当前 goal 和 feedback callback；
5. 记录最终退出原因；
6. 发布 `/gait/control_lock=false`；
7. 设置 `lock_owned=false`。

释放 lock 后：

- command_mux 会使所有旧命令缓存失效；
- 不得由 jump executor 重放旧 line 或 mission 命令；
- 必须等待新的 post-release 命令；
- jump Action 返回不等于 line-course 已恢复巡线。

## 7. Action feedback、result 和日志

### 7.1 feedback

在不修改 schema 的前提下，使用 `current_step` 表达监督阶段：

| 阶段 | `current_step` 示例 | `progress` 建议 |
|---|---|---|
| acquire | `start_jump: acquire_gait_lock` | 0.05 |
| zero publish | `start_jump: publish_locomotion_zero` | 0.10 |
| final zero wait | `start_jump: wait_final_cmd_zero` | 0.20 |
| SDK | `start_jump: sdk_front_jump` | 0.45 |
| settle | `start_jump: post_settle` | 0.70 |
| optional stability | `start_jump: stability_unavailable` | 0.85 |
| done | `start_jump: supervised_flow_done` | 1.00 |

finish profile 使用同样阶段名和 `finish_jump` 前缀。

feedback 只反映监督进度，不反映物理跳跃百分比。

### 7.2 result

成功：

- `success=true`
- message 明确监督流程完成和物理跨越未验证。

失败：

- `success=false`
- Action 状态为 aborted，message 包含阶段和稳定错误标识。

取消：

- `success=false`
- Action 状态为 canceled；
- message 说明取消发生阶段、SDK 是否可能已经发出以及最终零速保持。

现有 schema 无法 typed 表达 failure code 或 physical verification，这是明确接口缺口。
typed physical verification 必须作为后续独立接口升级，不扩大首个 PR。

### 7.3 结构化日志

每个 goal 至少记录：

```text
event
goal_id
motion_name
profile_name
supervision_stage
elapsed_sec
gait_lock_requested
locomotion_zero_published
final_zero_streak
final_zero_required
final_vx
final_vy
final_wz
cmd_mux_active_source
cmd_mux_reason
sdk_helper_path
sdk_network_interface
sdk_process_started
sdk_return_code
sdk_command_accepted
post_settle_completed
stability_check
physical_crossing_unverified
cancel_requested
estop_active
timeout_reason
cleanup_completed
```

不得在日志中输出秘密、无关环境变量或未验证的物理成功结论。

## 8. 取消与急停语义

FrontJump 真正发出后存在不可物理撤销窗口。

必须明确：

- 取消可阻止尚未启动的 helper；
- 取消可终止本地等待或 helper 进程；
- 取消可在动作后保持最终软件速度为零；
- 取消不能宣称中止半空中的动作；
- 急停可使 command_mux 软件输出归零；
- 急停不能保证撤销已经发给 SportClient 的 FrontJump；
- 发出 SDK 后收到 cancel/estop，仍必须以安全零速完成必要的本地收尾；
- 所有退出路径先保持零速，再释放 gait lock；
- 物理危险处置必须另有现场安全方案，不能只依赖软件 Action cancel。

## 9. 定向测试设计

### 9.1 测试替身

新增测试辅助对象：

- 可控的 fake clock；
- 捕获 lock 发布的 publisher；
- 捕获 locomotion Twist 的 publisher；
- 注入 `/navigation/cmd_vel` 的 final command probe；
- 可选 cmd_mux status probe；
- fake helper runner，可返回 0、非零、异常或阻塞；
- 可控 cancel 和 estop；
- 捕获 feedback、result 和结构化日志。

测试不得调用真实 SDK 或真实机器人。

### 9.2 必测用例

| 用例 | 核心断言 |
|---|---|
| start_jump 映射 | 选择 start profile 和 FRONT_JUMP executor |
| finish_jump 映射 | 选择 finish profile，且对象/值与 start 独立 |
| feedback 顺序 | 阶段顺序严格符合本文，progress 单调且在 `[0,1]` |
| lock 顺序 | 第一条 Jump 相关速度监督前先请求 gait lock，随后立即发布零 |
| 持续零命令 | zero wait、SDK、settle、异常和取消期间没有非零 Twist |
| 最终零确认 | 三轴均在 epsilon 内且连续 N 条新消息后才调用 helper |
| streak 重置 | 任一非零、非有限或陈旧样本使计数归零 |
| pre-stop 最小时长 | 即使很快得到 N 条零消息，也需满足 duration |
| final zero timeout | helper 未调用、result false、finally 清理 |
| cmd_mux status 辅助性 | status 缺失但真实 final zero 充分时可继续；status 不能替代 final zero |
| helper 路径缺失 | abort、无 SDK accepted、最终零、lock 释放 |
| SDK 返回 0 | 进入 settle，结果仍为 physical unverified |
| SDK 返回非零 | abort，不进入成功 done |
| helper 异常 | abort，记录异常并 finally |
| helper timeout | 终止/回收，abort，无遗留进程 |
| cancel before helper | helper 调用次数为 0 |
| cancel after helper start | 不宣称物理动作已取消，保持零并清理 |
| estop before helper | helper 调用次数为 0，abort |
| estop after helper start | 最终零，不宣称撤销 FrontJump |
| finally 清理 | 每种退出路径都先零后 unlock |
| 解锁缓存语义 | 解锁后旧 line/mission 命令不恢复，新命令才可动 |
| start/finish 独立参数 | 修改一方不影响另一方，分别使用各自 timeout/duration |
| 结果语义 | success 只表示 supervised flow，message 含 physical unverified |
| 当前稳定函数隔离 | 恒 True 的 `is_robot_stable()` 不产生 stability verified |

### 9.3 集成测试

在 VM 和 Go2 Foxy 分别验证：

1. 启动 gait node、command_mux 和 fake helper；
2. 发送 `start_jump` goal；
3. 观察 lock、locomotion candidate、最终 cmd、status 和 feedback；
4. 对 finish profile 重复；
5. 对 timeout、cancel 和 estop 重复；
6. 确认 `/navigation/cmd_vel` 只有 command_mux 一个发布者；
7. 确认解锁后旧候选不恢复。

## 10. 硬件验证等级

### 10.1 等级定义

| 等级 | 要求 | 允许结论 |
|---|---|---|
| 冒烟测试 | 同一障碍连续 5 次 | 仅 `SMOKE_TEST_PASSED` |
| 起点单项开发验收 | 起点障碍连续 20 次 | 起点障碍开发验收通过 |
| 终点单项开发验收 | 终点障碍连续 20 次 | 终点障碍开发验收通过 |
| 路线级验收 | 完整路线连续 10 场冷启动完成 | 路线级验收通过 |

禁止：

- 把连续 5 次写成单项开发验收；
- 把起点 10 次和终点 10 次合并成 20 次；
- 把台架 SDK 返回 0 计为实机成功；
- 只保留成功样本而删除失败样本。

### 10.2 条件分层

起点和终点分别统计：

- 高电量；
- 中等电量；
- 多档横向起跳偏差；
- 多档航向起跳偏差；
- 首次冷启动；
- 第二次比赛机会 reset 后运行；
- 不同地面摩擦和障碍固定状态；
- helper/网络接口和软件版本。

测试表必须保存每次尝试，而不是只保存聚合成功率。

## 11. 必须保存的证据

每次测试至少保存：

- Action goal；
- feedback 顺序和时间戳；
- Action result；
- SDK helper 绝对路径；
- 网卡参数；
- SDK stdout、stderr 和返回码；
- gait lock 状态；
- `/control/locomotion_cmd`；
- `/navigation/cmd_vel`；
- `/control/cmd_mux_status`；
- zero streak 和 epsilon；
- timeout 原因；
- cancel 原因和发生阶段；
- estop 时间；
- 机器人 IMU/姿态数据，如果存在真实数据源；
- 实机侧面视频；
- 实机俯视视频；
- 起点独立成功统计；
- 终点独立成功统计；
- 电量、横向偏差、航向偏差和 cold-start/reset 条件。

## 12. 首个 PR 完成定义

首个 PR 只有在以下条件全部满足时才可合并：

- start/finish 不再执行两段定时非零 Twist；
- 两个 motion_name 都调用共用 FRONT_JUMP executor；
- 起点和终点 executor 参数独立；
- lock 后立即并持续发布 locomotion 零；
- 最终软件速度连续归零后才启动 helper；
- cmd_mux status 仅作辅助；
- helper 成功、失败、异常、timeout 都有确定结果；
- cancel/estop 语义符合不可撤销窗口；
- post-settle 必经；
- 当前虚假稳定函数不被当作物理证据；
- 所有退出路径零速清理后释放 lock；
- 解锁后等待新命令；
- Action schema 未改变；
- result/log 明确 `physical_crossing_unverified`；
- 定向测试全部通过；
- 没有宣称硬件跨越成功。
