# 巡线、白色障碍、跳跃与启停区场景矩阵

## 1. 使用说明

本文把当前 ROS2 行为、省赛参考行为和目标行为放在同一矩阵中，用于后续 PR
评审、VM 测试、Go2 Foxy 联调和实机场地验收。

分类值只使用：

- `KEEP_CURRENT`
- `PORT_FROM_PROVINCIAL`
- `MERGE_AND_REWRITE`
- `REJECT`
- `NEEDS_FIELD_EVIDENCE`

实现成熟度标签：

- `[CURRENT]` 对应“当前行为”列，只描述仓库已存在行为；
- `[REFERENCE_ONLY]` 对应“省赛行为”列，只作静态参考；
- `[PROPOSED]` 对应“目标状态、目标输出、超时、失败和通过标准”；
- 所有待定速度、阈值、连续帧数和物理判据均为 `[NEEDS_FIELD_EVIDENCE]`。

约束：

- 完整 mission 赋予 `START_OBSTACLE`、`END_OBSTACLE` 业务语义；
- 不使用全局 `white_count` 判断第一条或第二条白线；
- line-course 只执行 mission 指定的巡线段；
- command_mux 始终是 `/navigation/cmd_vel` 唯一最终发布者；
- 表中“允许输出”指候选速度；最终速度仍由 command_mux 仲裁；
- VM 测试验证纯逻辑或 ROS 节点行为，Foxy 测试验证目标运行环境接口和时序，
  实机场景验证物理效果，三者不可互相替代。
- `sdk_command_accepted`、`post_settle_completed` 和
  `physical_crossing_unverified` 只允许作为结构化日志、现有
  `ExecuteMotion.Result.message` 文本或 `[PROPOSED]` RunMission 内部派生记录，
  不是当前 Action 字段。
- 终点 Jump 的 `[PROPOSED]` 上层状态统一为 `END_JUMP_SUPERVISED_DONE`，
  只表示受监督软件流程完成。

## 2. 场景矩阵

| 场景 | `[CURRENT]` 当前行为 | `[REFERENCE_ONLY]` 省赛行为 | 分类 | `[PROPOSED]` 目标状态 | 命令所有者 | 允许输出 | 超时处理 | 失败处理 | 所需日志 | VM 测试 | Foxy 测试 | 实机场景 | 通过标准 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 起点蓝区内待机 | line-course `WAIT_START` 输出零；完整 mission 默认参数仍可 auto-start，蓝区语义未与 RunMission 统一 | 程序启动后按固定 Phase 运行，没有正式本地 start goal 和蓝区就绪锁存 | `MERGE_AND_REWRITE` | `WAIT_START -> START_ZONE_ARMED` | 无运动源；mux 输出零 | 仅零 Twist | 等待 start 不超时；ready 超时只告警 | 节点未 ready 或检测陈旧时保持零 | run_id、ready 节点、蓝区新鲜度、start_latched=false | 注入蓝区且无 start，断言始终零 | 启动全栈但不发 goal，采样最终速度 | 狗完全位于起点蓝区，等待本地触发 | 未收到明确 start goal 时连续保持零 |
| 收到启动命令但没有新鲜 LineTrack | line follower 进入 `LINE_FOLLOW`，随后消息超时进入 STOP；缺少启动稳定 N 帧放行门 | Phase 1 前 20 帧强制零，但只按循环帧计数 | `MERGE_AND_REWRITE` | `START_COMMAND_LATCHED` | 无运动源；mux 输出零 | 仅零 Twist | `line_ready_timeout` | RunMission 失败或停在可重试状态，禁止起步 | LineTrack age、visible、confidence、ready streak、timeout | 无消息/旧消息启动测试 | Foxy 启动后延迟 tracker 发布 | 相机未就绪时触发本地 start | timeout 前后均无非零最终速度 |
| 正常直线 | lateral+heading 比例控制，横向误差小用 base speed | 各 Phase 对最大轮廓 offset 做 PID | `KEEP_CURRENT` | `TRACK` | line-course；mux 最终 | profile 限制内正向 vx 和小 wz | 段总 timeout | 停车并返回段失败 | 两类误差、confidence、suggested/final cmd、source | 合成直线输入 | 话题回放与 mux 集成 | 直线双向和不同光照 | 无振荡越界，持续选择同一线路 |
| 大横向误差 | 速度降到 slow，wz 限幅；没有独立停止阈值 | `abs(offset)>0.5` 降速；Phase 1 `>0.8` 停车 | `NEEDS_FIELD_EVIDENCE` | `TRACK_SLOW` 或 `ALIGN_TO_LINE` | line-course；mux 最终 | 低速 vx、限幅 wz；必要时零 vx | 收敛 timeout | 停车并报告 lateral_not_converged | lateral、速度档、限幅、收敛时间 | 参数边界测试 | 动态参数/话题回放 | 多个横向起始偏差 | 不出线，规定时间内收敛 |
| 大航向误差 | heading 参与 yaw，但线速度只按 lateral 调度 | 无独立 heading，只能从 offset 间接反应 | `MERGE_AND_REWRITE` | `TRACK_SLOW` | line-course；mux 最终 | 低速或零 vx、限幅 wz | heading 收敛 timeout | 停车并转入安全搜索或失败 | heading、lateral、降速原因 | 独立构造大 heading 小 lateral | Foxy 回放 LineTrack | 斜向压线起步 | 不以高速切入弯角，heading 达标后恢复 |
| 危险弯角 | tracker 给 corner candidate；line-course 定时预转，达到视觉条件或 max time 后重捕获 | Phase 1 特殊弯角逻辑被禁用；Phase 4 依赖 IMU 定角或固定帧数 | `MERGE_AND_REWRITE` | `CORNER_APPROACH -> CORNER_TURN -> REACQUIRE_LINE` | line-course；mux 最终 | 低速接近、原地/小半径转向 | 每阶段独立 timeout | 零速，段失败，不盲目继续 | corner 证据、方向、阶段、LineTrack、timeout | 状态机和方向测试 | detector+course+mux 回放 | 直角弯左右方向 | 阶段允许时才触发，转后稳定重捕获 |
| 弯角旁大黑块 | 多扫描带、宽度门、暗区拒绝和 route lock 可抑制误选，但阈值未覆盖正式场地 | 最大轮廓易吸附；面积突增、多分支有辅助信息 | `NEEDS_FIELD_EVIDENCE` | `TRACK` 或安全 `SEARCH_LINE` | line-course；mux 最终 | 可信线路存在时低速；否则零前进搜索 | 候选/搜索 timeout | 不把大黑块当新路线，失败则停车 | 候选数、选中带、route lock 拒绝原因、暗区比例 | 合成线+黑块图像 | 录包回放 | 大黑块位于内外弯侧 | route lock 不跳线且机器人不冲向黑块 |
| 短暂丢线 | `SHORT_LOST`，默认 vx=0，按最后方向小角速度恢复 | 多个 Phase 以 0.03/0.05 m/s 向前盲走 | `KEEP_CURRENT` | `SHORT_LOST_HOLD` | line-course；mux 最终 | 默认 vx=0，小 wz | `short_lost_timeout` | 转入定向搜索 | 丢线原因、last direction、age、cmd | 丢 1 至 N 帧测试 | 低频/丢包回放 | 短遮挡和断线 | 不向前冲，恢复后控制连续 |
| 转弯中丢线 | `TURN_LOST_KEEP` 保持最后转向方向，连续确认后恢复 | Phase 4 固定左转 30°；其他 Phase 搜索策略不统一 | `KEEP_CURRENT` | `TURN_LOST_KEEP -> REACQUIRE_LINE` | line-course；mux 最终 | vx=0，受限 wz | turn-lost timeout | 转入 sweep search，再超时停车 | last wz、expected direction、streak、age | 左右转丢线测试 | Foxy 状态话题检查 | 弯中遮挡 | 不反向抖动，重捕获必须连续稳定 |
| 长期丢线 | `TURN_90 -> SEARCH_LINE`，默认零前进；continuous search 可无限续期 | Phase 1 往复旋转，Phase 4 固定左转；部分阶段仍前进 | `MERGE_AND_REWRITE` | `SEARCH_LINE` | line-course；mux 最终 | 零 vx、受限扫描 wz | 段级硬 timeout，不能无限续期 | `EMERGENCY_STOP`/段失败 | 总搜索时间、扫描周期、方向、最终原因 | 总 timeout 测试 | 长时间无 LineTrack 联调 | 完全遮线/越线 | 到硬 timeout 必须稳定零速并失败 |
| LineTrack 消息超时 | follower 在 `line_msg_timeout` 后 STOP；line-course 对 suggested cmd 也有 timeout | 独占相机循环，无 ROS 消息 age 契约 | `KEEP_CURRENT` | `SAFE_STOP` | mux 输出零 | 仅零 Twist | 使用明确消息 age 阈值 | 记录 source stale，等待受控重启或重捕获 | header/receive age、timeout、mux source | 停止发布消息 | 杀死 tracker 节点 | 最迟在 `line_msg_timeout + 1 / command_mux_publish_rate + scheduling_margin` 内观察到最终软件零 Twist；不代表实体立即静止 |
| NaN/Inf 输入 | follower 拒绝非有限 LineTrack 并 STOP；line-course 拒绝非有限最终 cmd；mux 清空缓存 | 未见等价的完整非有限值边界 | `KEEP_CURRENT` | `SAFE_STOP` | mux 输出零 | 仅零 Twist | 立即，无等待 | abort 当前段，禁止自动恢复旧命令 | 字段名、原始类别、invalid count、mux reason | 各字段参数化 NaN/Inf | ROS 消息注入 | 不做动态实机注入；台架验证 | 无非有限命令到达 UDP，缓存失效 |
| 假重捕获 | SEARCH/TURN 状态有连续确认；但 STOP 收到单帧 trackable line 可直接回 `LINE_FOLLOW` | 多数 Phase 单帧 found 即恢复 | `MERGE_AND_REWRITE` | `REACQUIRE_LINE` | line-course；mux 最终 | 仅搜索/零速，不恢复巡线 vx | reacquire timeout | streak 清零；超时停车 | streak reset 原因、confidence、两类误差 | 真/假交替序列 | 话题脉冲注入 | 反光造成单帧黑线 | 单帧或不连续候选绝不恢复巡线 |
| 稳定重捕获 | SEARCH/TURN 使用 N 帧 visible、confidence 和 lateral 门限；未校验 heading | 省赛以连续居中思想或单帧 found 恢复，Phase 间不一致 | `MERGE_AND_REWRITE` | `REACQUIRE_LINE -> ALIGN_TO_LINE` | line-course；mux 最终 | 先搜索，再零/低速对正 | 两阶段独立 timeout | 任一阶段失败则停车 | 两个 streak、LineTrack age、误差、状态转换 | 连续 N 帧边界测试 | Foxy 回放 | 跳后和弯后找线 | 先重捕获再对正，heading/lateral 均连续达标 |
| 单帧白色障碍 | line-course 已有连续 3 帧确认，不触发；detector 单帧仍会发布候选 | 省赛需要 3 或 4 帧 | `KEEP_CURRENT` | 保持当前巡线段 | line-course；mux 最终 | 正常低风险巡线 cmd | 不适用 | 记录 candidate_rejected，不改变业务状态 | evidence、stage、confirm_streak=1 | 单帧图像测试 | 单消息注入 | 白色反光快速经过 | 不生成障碍事件 |
| 错误阶段白色障碍 | 当前 line-course 在任意 `LINE_FOLLOW` 都可能消费 | 只因 Phase 顺序自然限制，没有 typed 阶段门 | `MERGE_AND_REWRITE` | 保持当前 mission 阶段 | line-course；mux 最终 | 正常巡线或当前段允许 cmd | 不适用 | 以 stage_gate_rejected 记录，不锁存 | mission stage、candidate、拒绝原因 | 阶段组合测试 | mission+course 集成 | 非障碍区放置白色物 | 无 START/END 事件、无跳跃 |
| 持续白色障碍 | 连续确认后进入 `HANDLE_WHITE_BAR`，接近并等待外部 done；done 生产者当前未闭环 | 连续确认后进入居中和跳跃 | `MERGE_AND_REWRITE` | `APPROACH_OBSTACLE -> ALIGN_TO_LINE` | line-course；mux 最终 | 低速接近，达到位置门后零速 | approach/align timeout | 失败并保持零；不调用 Jump | semantic event、证据、center_y、streak、latch | 持续候选状态测试 | detector+course 回放 | 起点/终点分别测试 | 仅允许阶段触发一次并停在正确位置 |
| 同一障碍重复出现 | 只有全局 `white_bar_handled`，新 mission 会清空；不能区分两个障碍 | jump_count/Phase 状态避免部分重复，但依赖顺序计数 | `MERGE_AND_REWRITE` | 已完成事件保持 latched | mission 无新动作；mux 按当前状态 | 不允许再次 Jump | 不适用 | 忽略重复，持续零或继续恢复流程 | event_id、latched、duplicate count | 重放同一事件 | 重发检测消息 | 跳后仍看见原白障碍 | 同一 semantic event 只调用一次 Jump |
| 障碍离开后的重新装填 | 当前无显式离障 clear streak、cooldown 和 per-event rearm | Phase/cooldown_frames 局部实现，不能表达业务事件 | `MERGE_AND_REWRITE` | `EVENT_CLEARING -> REARMED` | line-course；mux 最终 | 重捕获/对正允许 cmd | clear/reacquire timeout | 不 rearm，返回失败或停车 | false streak、cooldown、rearm、mission stage | leave/re-enter 序列 | 消息回放 | 两处障碍之间完整段 | 未连续离障和切阶段前不能装填 |
| 跳前对正成功 | 当前 white handler 没有统一 lateral+heading 对正；gait jump 不读取 LineTrack | 省赛对 offset 连续 8 帧确认 | `MERGE_AND_REWRITE` | `ALIGN_TO_LINE -> FRONT_JUMP` | 对正由 line-course；Jump 由 locomotion；mux 最终 | 对正阶段零 vx、受限 wz/经验证 vy；Jump 阶段只允许 locomotion 零 Twist | align timeout | 未全部达标不得发 Jump goal | age、visible、confidence、lateral、heading、streak | 全条件门测试 | action 集成 | 多起跳偏差 | 所有条件连续 N 帧后只发一个 goal |
| 跳前对正丢线 | 当前无统一流程 | 省赛按 stored offset 转向，超时后强制跳 | `REJECT` 强制跳；目标为 `MERGE_AND_REWRITE` | `ALIGN_FAILED` | mux 输出零 | 仅零 Twist | align/reacquire timeout | 返回失败，禁止 FrontJump | loss age、stored direction、timeout、jump_not_sent | 丢线断言未调用 helper | Foxy action mock | 对正时遮线 | 最终零速且 helper 调用次数为 0 |
| 跳前对正超时 | 当前无统一流程 | Phase 1/5 直接跳 | `REJECT` 强制跳；目标为 `MERGE_AND_REWRITE` | `ALIGN_FAILED` | mux 输出零 | 仅零 Twist | 到期立即失败 | mission 决定有限重试或停止 | timeout、各门限未满足项、jump_not_sent | 虚拟时间超时测试 | Foxy action mock | 持续偏线 | 不调用 SDK，不报告 jump completed |
| Jump SDK helper 缺失 | 当前 jump 不调用 helper，因此不会发现该缺口 | legacy 进程内直接调用 SDK，无 helper 路径检查 | `MERGE_AND_REWRITE` | Action `ABORTED` | locomotion 持锁；mux 输出零 | 仅零 Twist | 立即路径检查失败 | finally 保持零并释放锁 | helper path、errno、stage、cleanup | 缺失路径测试 | 移除/重映射可执行文件 | 台架不发物理动作 | abort，锁释放，最终速度保持零 |
| Jump SDK 返回非零 | 当前两段 Twist 可返回 completed，没有 SDK 码 | legacy 打印失败码但时间结束后继续后续状态 | `MERGE_AND_REWRITE` | Action `ABORTED` | locomotion 持锁；mux 输出零 | 仅零 Twist | helper 正常退出即判断 | 非零码 abort，不进入成功语义 | SDK stdout/stderr、return code、profile | fake helper 返回 1 | Foxy fake executable | SDK 拒绝条件台架 | result success=false，无 completed 声明 |
| Jump SDK 超时 | 当前仅有动作总 timeout，不监督 SDK helper | legacy `FrontJump()` 同步调用，无独立进程 timeout | `MERGE_AND_REWRITE` | Action `ABORTED` | locomotion 持锁；mux 输出零 | 仅零 Twist | `sdk_timeout` 终止 helper | abort，post-failure zero，finally 清理 | pid、elapsed、timeout、terminate/kill 结果 | 阻塞 fake helper | Foxy subprocess 集成 | 不在无防护场景触发真实跳 | timeout 后无遗留进程、锁和非零 cmd |
| Jump Action 取消 | 当前 cancel 设置 emergency 标志并 stop；执行路径最终释放锁，但未定义 SDK 发出后的不可撤销语义 | legacy 仅键盘退出，不能取消已发 FrontJump | `MERGE_AND_REWRITE` | `CANCELED` 或 SDK 发出后的受控收尾 | locomotion 持锁；mux 输出零 | 仅零 Twist | 取消处理有上限 | 发出前阻止调用；发出后不宣称中止半空动作，保持零到 settle | cancel_at_stage、sdk_sent、cleanup | 每阶段 cancel 参数化测试 | Foxy Action cancel | 台架及受控实机 | 结果语义准确，任何取消点最终零速 |
| 急停 | mux estop 优先并清缓存；gait STOP 可中断循环，但不能撤回已发 SDK 动作 | legacy StopMove/退出，不能保证撤回 FrontJump | `KEEP_CURRENT` mux；Jump 语义 `MERGE_AND_REWRITE` | `EMERGENCY_STOP` | command_mux | 仅零 Twist | 软件验收上限为 `1 / command_mux_publish_rate + scheduling_margin` | 不宣称物理动作已撤销；保持锁/零到安全释放点 | estop time、stage、sdk_sent、final cmd、lock | estop 优先级测试 | SetBool/Bool 两入口 | 各阶段急停台架，实机按安全方案 | 在该上限内观察到最终软件零 Twist；不代表机器人实体或已发 FrontJump 立即停止 |
| 跳后重新找线 | 当前 gait 固定 recovery wait 后完成；white done 可进入 line-course reacquire，但调用链未闭合 | Phase 1 直接回巡线；Phase 5 固定直走 3 秒 | `MERGE_AND_REWRITE` | `POST_JUMP_SETTLE -> REACQUIRE_LINE` | line-course；mux 最终 | settle 仅零；搜索零 vx、受限 wz | settle 和 reacquire 独立 timeout | 失败停车，不固定前冲 | settle、LineTrack age、搜索方向、streak | 跳结果到搜索状态测试 | Action+LineTrack 回放 | 起点/终点跳后分别找线 | 不盲目前进，连续 N 帧后才结束重捕获 |
| 跳后二次对正 | 当前无统一状态 | Phase 3 有横移二次居中；Phase 1/5 不统一 | `MERGE_AND_REWRITE` | `ALIGN_TO_LINE -> RESUME_TRACK` | line-course；mux 最终 | 零/极低 vx、受限 wz；vy 需实证 | align timeout | 停车并返回失败 | lateral、heading、confidence、streak、vy/wz | 条件门和 timeout | Foxy 回放 | 多落地点偏差 | 两类误差连续达标后才恢复巡线 |
| 起点蓝区误触发 | 当前 line-course 任意 LINE_FOLLOW 均可进入 stop-zone 并最终停车 | legacy 无正式起终蓝区语义 | `MERGE_AND_REWRITE` | 起点阶段只到 `START_ZONE_ARMED` | 无最终停车动作 | 未 start 时零；已 start 后按离区策略 | 蓝区新鲜度 timeout | 不设置 final stop latch | phase、blue visible/inside、finish_armed=false | 起点蓝区消息测试 | 全栈起步回放 | 起点内摆动 | 起点蓝区绝不触发 `FINAL_STOP_LATCHED` |
| 起点离区锁存 | 当前没有 `START_ZONE_DEPARTED` | legacy 依赖 Phase 顺序 | `MERGE_AND_REWRITE` | `START_ZONE_DEPARTED` | line-course；mux 最终 | 正常起步 profile | departure timeout | 停车或任务失败，不提前开放终点逻辑 | clear streak、distance/phase、latch | 蓝区抖动离区序列 | stop-zone 话题集成 | 多种离区速度 | 连续多帧离区后锁存，之后抖动不撤销 |
| 终点停车未使能 | 当前任意 LINE_FOLLOW 可响应蓝区 | legacy 最终固定动作结束，没有阶段化蓝区使能 | `MERGE_AND_REWRITE` | 保持当前返航前阶段 | line-course；mux 最终 | 当前段巡线 cmd | 不适用 | 忽略蓝区并记录 gate reject | finish_armed、phase、blue candidate | gate=false 测试 | mission+course 回放 | 中途蓝色干扰 | 不减速、不锁存最终停车 |
| 终点停车已使能 | 当前 stop zone 连续确认后接近，inside 连续 3 帧即 FINAL_STOP | 无等价安全内边界和最终零速证明 | `MERGE_AND_REWRITE` | `APPROACH_FINISH_ZONE -> FINAL_STOP_CANDIDATE -> FINAL_STOP_LATCHED` | line-course 后由最终停车锁存；mux 最终 | 低速接近，候选成立后仅零 | approach/zero-confirm timeout | 零速失败锁存或 mission abort，不恢复旧 cmd | phase、armed、blue age、boundary、speed、zero streak | 全判据组合测试 | mux 最终零速集成 | 返回启停区 | 全部判据成立且零速保持至少 5 秒 |
| 蓝区检测抖动 | visible/inside 计数不连续会清零；进入 FINAL_STOP 后保持零 | 无正式蓝区处理 | `KEEP_CURRENT` 连续确认思想；整体 `MERGE_AND_REWRITE` | `APPROACH_FINISH_ZONE` 或已锁存 | line-course；mux 最终 | 未锁存时低速；锁存后仅零 | approach timeout | 未达连续门不停车；锁存后不解除 | visible/inside streak、reset reason | 抖动序列测试 | 消息注入 | 蓝区边缘往返 | 抖动不误锁存，已锁存不解除 |
| 蓝区检测陈旧 | APPROACH_STOP_ZONE 中变陈旧会 EMERGENCY_STOP | 无 ROS 新鲜度 | `KEEP_CURRENT` | `SAFE_STOP` | mux 输出零 | 仅零 Twist | detection age 超限立即处理 | 保持零，报告 stale，不靠旧 inside 状态停车 | receive age、header age、phase、cmd | 停止蓝区消息 | 杀 detector/相机 | 进入蓝区时断相机 | 旧消息不能完成最终停车判据 |
| 最终停车后残余命令 | line-course FINAL_STOP 周期发布零，mux mission 优先；缺少跨 RunMission 的正式 latch/reset 契约 | legacy StopMove 后进程结束，其他源隔离不明确 | `MERGE_AND_REWRITE` | `FINAL_STOP_LATCHED` | 最终停车锁存；mux 输出零 | 仅零 Twist | 零速确认和保持 timeout | 任一旧命令不得恢复；异常仍保持 estop/零 | 各 source fresh、mux reason、final cmd、latch | 注入旧/新候选命令 | 多发布者集成 | 停车后保留系统运行 | 5 秒及之后持续零，黑线/蓝区抖动不解锁 |
| 重复 mission start | line follower 每次 true 都重置到 LINE_FOLLOW；完整 mission 第二个 goal 在执行回调中 abort，但 start latch 未统一 | legacy 单进程单次运行 | `MERGE_AND_REWRITE` | 保持当前活动 RunMission | mission | 不新增运动或重置锁存 | 重复 goal 立即拒绝 | 返回 already_running，不改变 run_id/目标/障碍状态 | original/new goal id、active stage、reject reason | 并发 goal 测试 | Foxy Action 客户端双发 | 本地触发器按键抖动 | 只存在一个活动任务，状态不被重置 |
| 第二次比赛机会状态复位 | full mission 开始会 reset 部分上下文；line-course start 清部分计数；auto_goal_sent、最终停车、旧命令和所有业务锁存没有统一 reset 证据 | 重启进程可清内存，但不是受控第二次机会流程 | `MERGE_AND_REWRITE` | `SECOND_ATTEMPT_RESET -> LOCAL_BOOT_READY` | reset supervisor/mission；mux 保持零 | reset 期间仅零 Twist | reset readiness timeout | 不允许第二次 start，报告未清项 | 新 run_id、目标平台、START/END latch、final latch、mux caches | 两次连续 RunMission 测试 | Foxy 全栈 reset | 第一次失败后本地复位再运行 | 第二次不继承平台目标、障碍锁存、阶段或旧命令 |

## 3. 跨场景验收规则

### 3.1 VM

至少覆盖：

- tracker 合成图像：直线、弯角、大黑块、单帧/持续白条；
- line follower 纯消息序列：丢线、假重捕获、稳定重捕获、NaN/Inf、超时；
- line-course 状态：阶段门、事件锁存、离障重装填、蓝区使能；
- command_mux：锁优先级、消息过期、lock 释放缓存失效、最终零速；
- fake SDK helper：成功、非零、异常、缺失、超时、取消；
- 两次连续 RunMission 的上下文隔离。

### 3.2 Go2 Foxy

至少验证：

- Humble 侧设计没有使用 Foxy 不支持的 API；
- Action goal/feedback/result 与当前 `.action` 完全一致；
- `/navigation/cmd_vel` 只有 command_mux 一个发布者；
- gait lock 期间 mission/line 候选不能成为最终输出；
- lock 释放后旧缓存不恢复，必须收到新命令；
- 话题 QoS 和传感器频率足以支持连续 N 帧判据。

### 3.3 实机

所有实机测试先完成台架和低风险验证。参数统计必须区分：

- 起点障碍和终点障碍；
- 高电量和中等电量；
- 不同横向起跳偏差；
- 不同航向偏差；
- 第一次冷启动；
- 第二次比赛机会 reset 后运行。

连续 5 次只能标记 `SMOKE_TEST_PASSED`，不能作为单项开发验收。
