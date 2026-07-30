# 起点启停区、返航和最终停车验证计划

## 1. 目的与边界

本文设计起点就绪、明确启动、离开起点、终点障碍后返航、进入启停区和最终锁存
停车的业务状态与验证证据。

本轮只设计，不实现：

- 本地启动硬件；
- 本地急停硬件；
- 完整 RunMission 状态机；
- 蓝区 detector 改造；
- line-course 接口；
- 四足位置估计；
- 任何运行代码、参数或 ROS 接口。

起点蓝区和最终蓝区可能是同一物理区域，但业务语义必须由 mission 阶段决定，
不能仅依据“看到蓝色”。

### 1.1 语义标签

- `[CURRENT]`：当前 tracker、line-course、mission 或 command_mux 已存在的行为。
- `[PROPOSED]`：本文设计的状态、RunMission 上下文和脱机监督行为，当前尚未实现。
- `[NEEDS_FIELD_EVIDENCE]`：蓝区安全内边界、速度、连续帧数、epsilon、保持参数和
  四足入区结论，必须通过实机标定。

## 2. [CURRENT] 当前实现审计

### 2.1 已有能力

当前 tracker：

- 通过 HSV 和形态学检测蓝区；
- 发布 `SpecialTargetDetection`；
- 提供 visible、confidence、center、area、尺寸比例；
- 用相机参考点是否在蓝色轮廓内生成 `inside_candidate`。

当前 line-course：

- 对蓝区 visible 和 inside 都有连续帧确认；
- 检测陈旧时停车；
- approach 有 timeout；
- 进入 `FINAL_STOP` 后周期发布零 Twist。

当前 command_mux：

- 对候选命令有新鲜度；
- estop 优先；
- 最终输出非有限值会被拒绝；
- lock 释放时使旧缓存失效。

### 2.2 P0 缺口

- line-course 在任意 `LINE_FOLLOW` 都可能响应蓝区，没有起点/终点业务阶段门；
- 起点蓝区没有 `START_ZONE_ARMED` 语义；
- 完整 mission 默认参数 `auto_start=true`，不满足“未收到明确 start goal 时绝不移动”；
- start goal 没有统一、可审计的 RunMission latch；
- 没有 `START_ZONE_DEPARTED` 锁存；
- 没有 `FINISH_STOP_ARMED`；
- 单个相机参考点 inside 即可推进停车，尚无机器人尺寸和四足安全余量证据；
- 没有最终 `/navigation/cmd_vel` 连续归零确认；
- 没有零速保持至少 5 秒的验收状态；
- 没有跨普通 `/mission/stop` 的 `FINAL_STOP_LATCHED` 契约；
- 没有本地启动、节点 ready、本地急停和第二次机会 reset 的完整脱机方案；
- full mission、line-course、line follower 的 start/stop/reset 还没有统一 run_id。

## 3. [PROPOSED] 目标状态

主状态建议：

```text
WAIT_START
-> START_ZONE_ARMED
-> START_COMMAND_LATCHED
-> START_ZONE_DEPARTED
-> ...
-> END_JUMP_SUPERVISED_DONE
-> FINISH_STOP_ARMED
-> APPROACH_FINISH_ZONE
-> FINAL_STOP_CANDIDATE
-> FINAL_STOP_LATCHED
```

`END_JUMP_SUPERVISED_DONE` 只表示 FrontJump 受监督软件流程完成，不表示机器人
已经物理跨越或成功落地。

脱机运行监督转换：

```text
LOCAL_BOOT_READY
-> LOCAL_START_TRIGGER
-> AUTONOMOUS_RUNNING

AUTONOMOUS_RUNNING
-> LOCAL_EMERGENCY_STOP
-> SECOND_ATTEMPT_RESET
-> LOCAL_BOOT_READY
```

`LOCAL_EMERGENCY_STOP` 是异常分支，不是每次正常任务的必经状态。两组状态可由完整
mission/supervisor 组合管理，但不能由低层 detector 自行切换。

## 4. RunMission 上下文

一次活动任务至少持久保存：

```text
run_id
start_goal_latched
start_goal_latched_at
start_zone_armed
start_zone_departed
start_obstacle_completed
target_platform
target_platform_latched
end_obstacle_completed
end_jump_supervised_result
post_jump_route_ready
finish_stop_armed
final_stop_candidate_since
final_zero_confirm_streak
final_zero_hold_started_at
final_stop_latched
failure_reason
```

规则：

- 新 RunMission goal 创建新的 `run_id`；
- 同一活动任务内重复 start 不改变上下文；
- 普通检测消息不得改写已锁存字段；
- 未锁存目标平台时不得默认选择平台；
- 新任务/reset 前最终停车保持最高业务优先级；
- 节点重启后不得自动恢复旧任务为运行状态。

## 5. 起点设计

### 5.1 WAIT_START

进入条件：

- 本地正式程序已经启动；
- mission 尚未收到有效 start goal；
- 或上一次任务已 reset 且当前无活动 run。

行为：

- 不允许 line-course 发布非零 mission candidate；
- command_mux 最终输出必须为零；
- 可持续接收 tracker、蓝区和节点 ready 状态；
- 不因为黑线、白色障碍或蓝区检测而自动移动。

禁止以当前 `auto_start=true` 作为正式比赛启动方式。正式配置必须在后续实现 PR
中改为明确的本地 start trigger 产生 RunMission goal；本轮只记录缺口。

### 5.2 START_ZONE_ARMED

起点蓝区只表示起跑前就绪。

候选条件：

- 当前业务状态为 `WAIT_START`；
- 蓝区检测消息新鲜；
- visible 和 confidence 连续 N 帧达标；
- 相机参考点满足起点就绪 profile；
- 必要节点都处于 ready；
- estop 未激活。

进入后仍保持零速。该状态：

- 不触发最终停车；
- 不设置 `FINISH_STOP_ARMED`；
- 不消费终点蓝区事件；
- 不自行生成 start goal。

蓝区检测抖动可使“当前可见”状态变化，但不允许机器人移动。

### 5.3 START_COMMAND_LATCHED

唯一合法触发：

- 收到有效、明确、合规的本地 RunMission start goal；
- 当前没有其他活动 RunMission；
- 系统 ready 检查通过。

锁存：

- 保存 `run_id` 和 goal id；
- `start_goal_latched=true`；
- 重复本地触发或按键抖动只记录 duplicate；
- 不重新初始化正在运行的阶段；
- 不清除已锁存的平台目标或障碍状态。

收到 start 后仍不能立即运动。必须通过：

- tracker 消息新鲜；
- `line_visible=true`；
- confidence 达标；
- lateral、heading 有限；
- 连续 N 帧稳定；
- command_mux、gait lock、arm lock 和 estop 允许。

ready timeout 时保持零速并返回明确失败，不用旧 LineTrack 起步。

### 5.4 START_ZONE_DEPARTED

离开起点的候选条件：

- 当前 run 的 start goal 已锁存；
- mission 已进入离开起点的合法阶段；
- 蓝区消息新鲜；
- 连续 M 帧满足“不在起点蓝区安全内部”；
- 不能仅用单帧 `visible=false`；
- 不能把相机断流当成已离区。

锁存后：

- `start_zone_departed=true`；
- 普通蓝区抖动不得清除；
- 后续重新看到蓝区不回到 `START_ZONE_ARMED`；
- 在其完成前禁止任何最终停车逻辑；
- 只有新 RunMission/reset 才清除。

离区 timeout：

- 降速并停车；
- 返回 `start_zone_departure_timeout`；
- 不把固定时间向前运动当成成功离区。

## 6. 起点和终点白色障碍

完整 mission 只在对应阶段消费：

| 阶段 | 事件 |
|---|---|
| `FOLLOW_TO_START_OBSTACLE` | `START_OBSTACLE` |
| `FOLLOW_TO_END_OBSTACLE` | `END_OBSTACLE` |

禁止使用全局 `white_count`、障碍出现次序或累计次数决定起点/终点业务语义；
白色候选只有在当前 mission 阶段允许时才能被确认和消费。

起点障碍完成后锁存 `start_obstacle_completed=true`。

终点障碍只有在监督 Jump Action 返回后才可更新：

```text
end_jump_supervised_result.sdk_command_accepted=true
end_jump_supervised_result.post_settle_completed=true
end_jump_supervised_result.physical_crossing_unverified=true
```

以上三个名称是 `[PROPOSED]` RunMission 内部派生记录，不是当前
`ExecuteMotion.action` 字段。当前 Action 的真实字段始终是：

- Goal：`motion_name`
- Result：`success`、`message`
- Feedback：`current_step`、`progress`

该内部记录只能从现有 result `success/message` 和对应结构化日志派生，不能被当成
新的跨节点安全协议。首个 FrontJump PR 不修改 mission 或 Action schema。

`physical_crossing_unverified` 不阻止软件流程记录“受监督调用已完成”，但禁止把它
写成实体跨越成功。是否继续返航由 mission 的比赛策略决定，并需要实机验证。

## 7. 终点停车使能

只有以下条件全部满足，mission 才能设置：

```text
FINISH_STOP_ARMED=true
```

前置条件：

- 当前活动 RunMission 有效；
- `START_ZONE_DEPARTED=true`；
- 终点障碍业务事件已完成；
- Jump Action 已返回受监督执行结果；
- SDK 调用和 post-settle 结果已记录；
- 跳后已通过 `REACQUIRE_LINE + ALIGN_TO_LINE`，或 mission 明确进入经批准的返航模式；
- 当前状态已进入 `RETURN_START_ZONE`；
- estop 未处于需要人工 reset 的状态。

任一条件缺失：

- 蓝区候选只能记录；
- 不能减速到最终停车逻辑；
- 不能设置 `FINAL_STOP_CANDIDATE`；
- 不能依据“再次看到起点蓝区”自行推断比赛完成。

`FINISH_STOP_ARMED` 是 RunMission 锁存，只能由新任务/reset 清除。

## 8. APPROACH_FINISH_ZONE

进入条件：

- 当前阶段为 `RETURN_START_ZONE`；
- `FINISH_STOP_ARMED=true`；
- 收到新鲜、连续稳定的蓝区候选。

行为：

- 使用独立 finish-zone 低速 profile；
- 继续通过 line-course 和 command_mux 输出；
- 不绕过 mux；
- 接近速度必须低于普通返航速度；
- 检测陈旧、非有限或冲突时立即输出零；
- approach 有硬 timeout。

蓝区 visible 只允许启动低速接近，不等于已经完全进入启停区。

## 9. FINAL_STOP_CANDIDATE

### 9.1 必须同时满足的候选条件

- 当前阶段为 `RETURN_START_ZONE`；
- `FINISH_STOP_ARMED=true`；
- `START_ZONE_DEPARTED=true`；
- 蓝区检测消息新鲜；
- 蓝区连续稳定可见；
- confidence 达标；
- 相机参考点越过经实机标定的安全内边界；
- 接近速度已经降低；
- 所有输入值有限；
- 没有活动 gait/arm 动作；
- estop 状态与最终停车策略一致。

单个相机参考点只能作为候选条件，不能单独证明四个足端都在蓝区。

### 9.2 安全内边界

安全内边界必须比蓝区实际边缘更靠内，预留：

- 相机安装偏移；
- 机身前后长度；
- 机身左右宽度；
- 四足站立外廓；
- 制动距离；
- 视觉误差；
- 蓝区边缘磨损和光照变化；
- 控制、消息和 mux 延迟。

边界标定流程：

1. 标定相机内外参和地面投影；
2. 建立相机参考点到机器人机身坐标的变换；
3. 建立机器人机身和四足最大安全外廓模型；
4. 以低速从多种横向/航向偏差进入蓝区；
5. 同步保存 detector 输出和最终 cmd；
6. 用俯视视频标注四个足端位置；
7. 计算参考点阈值与真实足端余量；
8. 选择覆盖失败样本和安全余量的内边界；
9. 独立验证，不使用同一批数据既标定又验收。

该阈值属于 `NEEDS_FIELD_EVIDENCE`。

### 9.3 最终软件零速确认

进入 candidate 后持续向 mission candidate 发布零，并订阅：

```text
/navigation/cmd_vel
```

单条消息归零条件：

```text
abs(linear.x)  <= final_stop_zero_epsilon
abs(linear.y)  <= final_stop_zero_epsilon
abs(angular.z) <= final_stop_zero_epsilon
```

要求：

- 三个值有限；
- 连续 N 条新收到的消息；
- 消息在当前候选窗口内；
- 非零、非有限或陈旧使 streak 归零；
- 有总 zero-confirm timeout；
- cmd_mux status 只作为 active source、reason 和 freshness 辅助日志。

最终软件速度为零不等于物理完全静止。

### 9.4 零速保持

连续归零后开始保持计时：

- 最终软件命令持续归零；
- 最短保持 5 秒；
- 期间不得释放最终停车业务锁；
- 任一非零最终命令使保持计时归零并记录；
- 蓝区检测抖动不应导致恢复运动；
- 如果检测陈旧，继续保持零，不以旧检测完成新的空间证明；
- hold timeout 或异常时保持零并进入安全失败，不恢复返航。

5 秒完成后才进入 `FINAL_STOP_LATCHED`。

## 10. FINAL_STOP_LATCHED

进入后：

- 最终速度持续为零；
- 普通蓝区检测抖动不得解除；
- 重新看到黑线不得解除；
- 旧 mission、line 或 locomotion 命令不得恢复；
- 普通 `/mission/stop` 不得将其错误恢复为运行状态；
- 重复 start trigger 不得解除；
- detector 重启不解除；
- action 结果迟到不解除；
- 记录完成 run_id 和锁存时间。

只允许以下操作复位：

- 明确的新 RunMission goal，在完成完整 reset 后创建新 run_id；
- 明确的 `SECOND_ATTEMPT_RESET`；
- 经定义的维护/重启流程。

新任务 reset 必须先确认最终软件速度仍为零，并清除旧 command_mux 缓存，再允许
重新进入 `WAIT_START`。

## 11. 失败与超时策略

| 场景 | 目标行为 |
|---|---|
| start 前节点不 ready | 保持零，不接受运行 |
| start 后 LineTrack 不新鲜 | ready timeout，任务失败或停在可重试状态 |
| 离区检测陈旧 | 不锁存 departed，停车 |
| 终点障碍未完成却看到蓝区 | 阶段门拒绝，不进入最终停车 |
| Jump 受监督流程失败 | 不设置 finish arm，按 mission 策略停止或有限重试 |
| 跳后未找到线 | 不设置 finish arm，停车 |
| 返航中蓝区丢失 | 降到零，等待新鲜检测或 approach timeout |
| 安全内边界未满足 | 保持低速接近，达到 timeout 后停车失败 |
| 最终零速未连续确认 | 保持零候选，timeout 后安全失败 |
| 5 秒保持期出现非零最终命令 | 重置 hold timer，记录源；持续异常则 estop |
| FINAL_STOP_LATCHED 后收到旧命令 | mux/最终锁存继续输出零并记录违规源 |

失败状态不能通过固定时间自动转为成功。

## 12. 脱离电脑运行的 P0 设计缺口

赛规要求机器人在正式比赛过程中脱线运行。当前脚本可启动节点并可手工发布
`/mission/start`，但尚未形成以下经验证的本地闭环。

### 12.1 LOCAL_BOOT_READY

必须在机器人本地完成：

- 系统启动正式程序；
- 校验配置和正式 launch；
- 确认 perception、navigation、mission、locomotion、mux、SDK bridge ready；
- 确认 `/navigation/cmd_vel` 唯一发布者为 command_mux；
- 确认相机和关键消息新鲜；
- 确认 estop 通道可用；
- 未满足时保持零并给出本地可观察错误。

### 12.2 LOCAL_START_TRIGGER

必须是赛规允许的机器人本地触发：

- 只生成一个明确 RunMission goal；
- 有去抖和单次锁存；
- 不能依赖 SSH、开发电脑终端或开发电脑网络；
- 不能绕过 mission 直接发布速度；
- 具体按钮、遥控器或其他硬件方案留待独立设计评审。

### 12.3 AUTONOMOUS_RUNNING

- SSH 断开不影响节点；
- 开发电脑网络断开不影响相机、DDS、SDK 或 mission；
- 本地日志持续保存；
- 任务完成后本地进入 `FINAL_STOP_LATCHED`；
- 不因远程客户端消失而取消任务，除非赛规和安全设计明确要求。

### 12.4 LOCAL_EMERGENCY_STOP

- 必须有合规的本地触发；
- 进入 command_mux estop；
- 最终软件速度归零；
- 不依赖 SSH；
- 对已发 FrontJump 只声明软件零速，不能承诺撤销物理动作；
- estop 清除必须经过明确 reset，不能自动恢复旧命令。

### 12.5 SECOND_ATTEMPT_RESET

第一次机会失败后，本地 reset 必须：

- 保持最终零速；
- 终止/回收活动 Action 和 helper；
- 清除 command_mux 旧候选；
- 创建新的 run_id；
- 清除活动 mission 阶段；
- 清除平台目标及其确认 streak；
- 清除 START/END 障碍锁存和重装填状态；
- 清除 start goal 和 start-zone-departed 锁存；
- 清除 finish arm、candidate、zero streak、hold timer 和 final latch；
- 清除丢线、重捕获和对正 streak；
- 不继承旧检测消息；
- 重新执行 LOCAL_BOOT_READY；
- 等待新的 LOCAL_START_TRIGGER。

本轮不选择具体本地启动硬件，只记录行为契约和验收缺口。

## 13. 脱机正式验证流程

每次正式验收：

1. 机器狗本地启动正式比赛程序；
2. 本地确认所有节点 ready；
3. 记录 branch/build/config 标识；
4. 断开 SSH；
5. 断开开发电脑网络；
6. 等待至少一个完整检测新鲜度窗口，确认系统不依赖电脑；
7. 使用合规的本地启动触发；
8. 机器人独立完成任务；
9. 返回启停区并本地锁存停车；
10. 验证最终软件速度连续归零并保持至少 5 秒；
11. 保留本地日志和视频；
12. 对第一次机会失败的场景执行本地安全 reset；
13. 使用第二次本地触发重新运行；
14. 对比两个 run_id，确认没有继承第一次状态。

任何步骤仍需 SSH 命令才能继续，都不能标记为脱机验收通过。

## 14. 验证矩阵

### 14.1 VM

| 测试 | 注入 | 通过标准 |
|---|---|---|
| 无 start 不移动 | 新鲜蓝区、黑线和候选速度 | 最终速度始终零 |
| start goal 去抖 | 连续多个相同 start | 只创建一个 run_id |
| tracker ready 门 | 旧、低置信度和稳定 LineTrack | 只有连续稳定输入后允许起步 |
| 起点离区 | visible/inside 抖动和连续离区 | 只在连续 M 帧后锁存 |
| 中途蓝区 | finish arm=false 时持续蓝区 | 不进入 final stop |
| 终点使能 | 逐项补齐前置条件 | 仅全部满足时 arm |
| safe boundary | 边界内外候选序列 | 单个 inside 不足以完成停车 |
| final zero | 零/非零/NaN/陈旧序列 | 连续 N 条新鲜零消息才通过 |
| 5 秒保持 | 中途插入非零 | timer 归零，不能提前 latch |
| latch 抗干扰 | 黑线、蓝区抖动、旧命令 | 始终保持零 |
| 普通 mission stop | latch 后发布 stop | 不恢复运行 |
| 第二次机会 | 两次 run 和 reset | 所有列出的上下文都隔离 |

### 14.2 Go2 Foxy

验证：

- 当前消息和 Action schema 兼容；
- 最终 cmd 只有 command_mux 发布；
- 节点 kill、相机断流和消息 timeout 会归零；
- start trigger 不依赖开发电脑；
- estop 的 Bool/SetBool 入口都到同一 mux 状态；
- reset 后旧 source freshness 为 false；
- 本地日志包含 run_id 和状态转换。

### 14.3 实机

起点：

- 多种初始站位；
- 蓝区边缘和内部；
- start 按键抖动；
- 不同 LineTrack 就绪延迟；
- 连续离区确认。

最终停车：

- 不同返航横向和航向偏差；
- 高电量和中等电量；
- 蓝区边缘磨损和照明变化；
- 不同接近速度；
- 单相机参考点进入但后足仍在区外的反例；
- 四足完全入区的正例；
- 锁存后至少 5 秒以及更长观察；
- 第一次失败后第二次机会。

## 15. 必须保存的证据

- RunMission goal、run_id、feedback 和 result；
- start trigger 时间和去抖结果；
- 节点 ready 清单；
- SSH/开发电脑网络断开时间；
- 蓝区原始消息和 receive age；
- `inside_candidate`、center、confidence、area；
- 安全内边界参数和标定版本；
- line-course 状态和 mission 阶段；
- `FINISH_STOP_ARMED`；
- `/control/mission_cmd`；
- `/navigation/cmd_vel`；
- `/control/cmd_mux_status`；
- final zero streak；
- 5 秒 hold 的开始、重置和完成时间；
- `FINAL_STOP_LATCHED` 时间；
- 相机标定；
- 机器人尺寸和四足外廓模型；
- 俯视视频；
- 侧面视频；
- 每次实机尝试的电量、偏差和结果；
- 第二次机会 reset 前后上下文快照。

## 16. 通过标准

### 16.1 起点

- 起点蓝区只产生 ready 语义；
- 无明确 start goal 永不移动；
- 重复 start 不重启任务；
- 没有新鲜稳定 LineTrack 时不移动；
- 连续离区后 `START_ZONE_DEPARTED` 锁存；
- 离区前最终停车逻辑不可达。

### 16.2 最终停车

- 只有终点障碍和跳后恢复完成后才能 arm；
- 蓝区消息新鲜且连续；
- 相机参考点越过已验证安全内边界；
- 接近速度已降低；
- 最终软件速度连续归零；
- 零速保持至少 5 秒；
- final latch 后任何普通感知、旧命令或 `/mission/stop` 都不能恢复运动；
- 只有新任务/reset 能复位。

### 16.3 脱机与第二次机会

- 断开 SSH 和开发电脑网络后可独立运行；
- 本地 start 和本地 estop 可用；
- 完成后本地锁存停车；
- 第一次机会失败后可本地安全 reset；
- 第二次机会不继承第一次的任务阶段、平台目标、障碍锁存、最终停车锁存或旧命令。
