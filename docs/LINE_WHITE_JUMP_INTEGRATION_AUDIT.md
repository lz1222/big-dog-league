# ROS2 与省赛巡线、白色障碍、跳跃整合审计

## 1. 文档目的与边界

本文审计当前国赛 ROS2 主体与
`legacy/provincial_reference/final_provincial_reference.py` 中可复用的省赛实机经验，
并给出后续整合设计。当前 ROS2 工程始终是唯一正式架构；legacy 只作为静态证据，
不运行、不安装、不进入 launch 或 console entry point。

本轮只形成设计，不修改运行代码、参数、消息、Action、Service、launch、脚本或
Unitree UDP 协议。

不在本轮处理的模块：

- 迷宫和避障区的正式实现；
- 楼梯动作；
- 机械臂、抓取、中转和放置动作；
- 各平台的精确对准；
- 完整赛道状态机落地。

冻结项：

- 检测平台三个警示标识的识别算法；
- 当心触电到伸懒腰、当心强氧化物到打招呼、当心辐射到前灯闪烁三次的映射；
- `LineTrack` 现有接口；
- `command_mux` 对 `/navigation/cmd_vel` 的唯一最终控制权。

### 1.1 语义标签

- `[CURRENT]`：已在当前仓库运行代码或真实 ROS 接口中存在并经静态审计确认。
- `[PROPOSED]`：本文设计的目标行为、状态、内部记录或后续 PR 内容，当前尚未实现。
- `[NEEDS_FIELD_EVIDENCE]`：必须通过 Foxy/实机数据标定后才能定值或宣称有效。
- legacy 内容统一视为 `[REFERENCE_ONLY]`，不属于当前运行架构。

`KEEP_CURRENT` 等五类标签表示候选逻辑的处置结论，不替代上述实现成熟度标签。
下文中的“当前”均为 `[CURRENT]`；“目标、建议、后续、应当”均为
`[PROPOSED]`，除非明确标为 `[NEEDS_FIELD_EVIDENCE]`。

## 2. 审计来源与当前控制链

主要静态证据：

- `src/rk_perception/rk_perception/real_line_tracker_node.py`
- `src/rk_navigation/rk_navigation/line_follower_node.py`
- `src/rk_mission/rk_mission/mission_state_machine_node.py`
- `src/rk_locomotion/rk_locomotion/gait_control_node.py`
- `src/rk_safety/rk_safety/command_mux_core.py`
- `src/rk_go2_sdk_bridge/src/go2_sdk_motion_action.cpp`
- `src/rk_interfaces/msg/LineTrack.msg`
- `src/rk_interfaces/msg/SpecialTargetDetection.msg`
- `src/rk_interfaces/msg/SignDetection.msg`
- `src/rk_interfaces/action/ExecuteMotion.action`
- `legacy/provincial_reference/final_provincial_reference.py`

正式速度链保持为：

```text
LineTrack
-> line_follower suggested cmd
-> line_course mission
-> /control/mission_cmd
-> command_mux
-> /navigation/cmd_vel
-> UDP/SDK 执行层
```

任何移植经验都必须进入上述链路。legacy 的 `SportClient.Move()`、相机循环和
`IntegratedMission` 不得成为正式控制者。

## 3. 总体结论

| 模块 | 结论 | 理由 |
|---|---|---|
| 当前多扫描带选线与 route lock | `KEEP_CURRENT` | 能输出稳定的横向误差、航向误差、置信度和可见性，并抑制跨候选跳变 |
| 当前 suggested-command 与 mux 链 | `KEEP_CURRENT` | 保持分层职责和最终速度单一所有者 |
| 省赛 PID、启动保护和分阶段参数 | `MERGE_AND_REWRITE` | 经验有价值，但不能复制其相机、控制线程和直接 SDK 架构 |
| 白色障碍双证据 | `MERGE_AND_REWRITE` | 当前横向亮条证据应保留，省赛中央 ROI 行占比和黑线间隙只作为附加证据 |
| FrontJump 动作语义与停稳思想 | `PORT_FROM_PROVINCIAL` | 在现有 ROS Action、gait lock、mux 和 SDK2 helper 内重写 |
| 省赛按第一次/第二次白线决定业务 | `REJECT` | 起点和终点必须由 mission 阶段赋予语义 |
| 省赛丢线后低速盲目前进 | `REJECT` | 普通丢线不得被当作白线或到达证据 |
| 跳前、跳后横移对正 | `NEEDS_FIELD_EVIDENCE` | `LineTrack` 可提供误差，但 `vy` 方向、增益和安全余量需要实机证明 |

## 4. A：当前与省赛巡线控制比较

### 4.1 视觉与控制输入

当前 tracker：

- 对二值图执行多扫描带候选提取；
- 选择跨扫描带连续的最佳路径，不以单个最大轮廓作为线路；
- 计算归一化 `lateral_error`；
- 根据路径上下位置关系计算 `heading_error`；
- 以有效扫描带数量形成 `confidence`；
- 通过 route lock、候选跳变拒绝和稳定切换避免突然切到邻近黑色目标；
- 发布 `line_visible`，拒绝过暗、路径带数不足、置信度低或异常候选。

省赛 `LineDetector.detect()`：

- 只处理画面下三分之一；
- 从外轮廓中选择面积最大的有效轮廓；
- 使用该轮廓质心生成单一 offset；
- 同时保留最大轮廓面积、显著轮廓数量、左右触边信息；
- Phase 1、3、4、5 分别切换阈值、最小面积和 PID 参数。

结论：当前多扫描带路径和 route lock 更适合国赛主线，应保留；面积突增和多分支
数量只能作为特殊弯角候选特征研究，不得替换当前 tracker。

### 4.2 逐项差异

| 比较项 | 当前 ROS2 | 省赛各 Phase | 目标结论 |
|---|---|---|---|
| PID 结构 | `wz = -(kp_lateral*lateral + kp_heading*heading)`；本质为双误差比例控制，可选角速度平滑和死区 | 离散 PID，含 I、D、梯形积分、积分限幅和输出饱和；各 Phase 使用不同参数 | `KEEP_CURRENT` 主结构；I/D 是否引入为 `NEEDS_FIELD_EVIDENCE` |
| 启动保护 | 收到 `/mission/start` 后立即进入 `LINE_FOLLOW`；无消息最终由超时停车，但没有“新鲜且稳定 N 帧后才放行”的门 | Phase 1 前 20 帧强制零速，首个可信 offset 只初始化 PID，避免 D 项冲击 | `MERGE_AND_REWRITE` 为显式 `START_READY` 门 |
| 速度调度 | 按 `abs(lateral_error)` 三级调度 base/mid/slow；不直接按航向误差或置信度降速 | 各 Phase 固定基础速度，大 offset 时乘 0.6；另有 SpeedLevel | `MERGE_AND_REWRITE`，保留当前分级并加入危险航向/低置信度约束 |
| 大横向误差 | 降到 slow speed，角速度受上限约束；非有限值停车 | Phase 1 对 `abs(offset)>0.80` 直接停车；通常在 `>0.5` 时降速 | 阈值与是否停车为 `NEEDS_FIELD_EVIDENCE` |
| 航向误差 | 是控制输入，可区分“偏在线旁”与“线路朝向不对” | 最大轮廓质心 offset 无独立航向误差 | `KEEP_CURRENT` |
| 直线 | 横向和航向误差共同收敛；横向误差小则使用 base speed | PID 跟随 offset，使用 Phase 固定速度 | 当前结构 `KEEP_CURRENT`，速度值待实证 |
| 缓弯 | 路径斜率形成 heading，连续调节 yaw；route lock 保持目标 | PID 只根据质心偏移转向 | `KEEP_CURRENT` |
| 直角弯 | tracker 发布 corner candidate；line-course 可进入定时 `CORNER_PRE_TURN`，随后重捕获 | Phase 1 曾设计面积突增、多分支、前冲里程和 IMU 转角，但当前 `is_sharp_turn=False`；Phase 4 有 IMU 定角转向 | `MERGE_AND_REWRITE`；阶段允许、视觉证据、超时和重捕获必须闭环 |
| 大黑块旁弯角 | 多扫描带、宽度限制、暗区比例限制和 route lock 降低吸附风险 | 最大轮廓可能被大黑块吸引；面积突增、多分支或全画面黑占比用于特殊状态 | 当前 tracker `KEEP_CURRENT`；特殊提示需 `NEEDS_FIELD_EVIDENCE` |
| 短暂丢线 | `SHORT_LOST`；默认前进速度为零，按最后方向小角速度找线 | 多个 Phase 在丢线时以 0.03 或 0.05 m/s 继续前进 | 当前停前进策略 `KEEP_CURRENT`；省赛盲目前进 `REJECT` |
| 转弯中丢线 | 根据最后 yaw 进入 `TURN_LOST_KEEP`，默认原地保持转向并连续确认重捕获 | Phase 4 丢线后站立、执行动作并按 IMU 左转；其他 Phase 多为固定方向搜索 | 当前按转弯历史恢复 `KEEP_CURRENT`，场景化转向策略需重写 |
| 长期丢线 | `TURN_90` 后 `SEARCH_LINE` 往复搜索；默认线速度为零；可持续搜索 | Phase 1 30 帧后左右摆转；部分 Phase 长期以低速前进；Phase 4 固定左转 30° | 不允许无限无边界动作；增加段级总超时后安全失败 |
| 消息超时 | `line_msg_timeout` 后进入 `STOP` 并发布零；非有限输入也停车 | 独占相机循环，没有 ROS 消息新鲜度契约；无帧时多为继续等待 | `KEEP_CURRENT` |
| 动作后重新找线 | line follower 有搜索/稳定确认；line-course 有 `REACQUIRE_LINE`，但当前 Jump 动作未与它闭环 | Phase 1 跳后直接回 NORMAL；Phase 3 强制右移搜索；Phase 5 跳后固定直走 | `MERGE_AND_REWRITE` 为统一 `REACQUIRE_LINE` |
| 动作后重新居中 | 当前没有独立、同时约束 lateral 与 heading 的统一状态 | Phase 3 使用 `vy` 连续居中；Phase 1/5 用 yaw 对 offset 连续确认 | `MERGE_AND_REWRITE` 为统一 `ALIGN_TO_LINE` |

### 4.3 省赛 Phase 经验的适用性

#### Phase 1

可借鉴：

- 启动帧保护；
- 首次可信误差只用于控制器初始化；
- 白线连续帧确认；
- 白线后先对正、停顿，再调用 FrontJump；
- 跳后等待和重新进入巡线的阶段划分。

拒绝：

- 用 `jump_count` 和“第二次白线”赋予业务语义；
- 丢线或对正超时后强制跳；
- 普通丢线后继续向前；
- 直接通过 odometry、IMU 和 `SportClient.Move()` 控制正式机器人。

#### Phase 3

可借鉴：

- 动作前、动作后分别居中；
- 连续居中确认；
- 动作后主动找线和再次居中的明确阶段。

需要实机证据：

- 使用 `linear.y` 横移；
- 横移方向、增益、最大速度；
- 强制单向横移固定帧数。

#### Phase 4

可借鉴：

- 转向或动作后重置控制器；
- 动作结束后重新建立相机与线路状态的思想。

拒绝：

- 固定巡线时长作为到达证据；
- 丢线后固定向前；
- 用固定时间、固定角度或固定帧数冒充任务成功。

#### Phase 5

可借鉴：

- 终点障碍使用独立白线阈值和跳跃参数的经验；
- 跳前连续居中、预停和动作后等待。

拒绝：

- 对正丢线或超时后直接跳；
- 跳后固定直走 3 秒即认为恢复；
- 跳后固定左转和横移作为通用返航方案。

## 5. B：当前 LineTrack 的能力边界

### 5.1 能直接支持

| 能力 | 使用方式 |
|---|---|
| `lateral_error` | 巡线 yaw、跳前横向居中候选条件、跳后居中 |
| `heading_error` | 巡线 yaw、危险弯角减速、跳前和跳后二次对正 |
| `confidence` | 速度调度、对正放行、重捕获确认 |
| `line_visible` | 区分可跟踪、搜索和失败；不得单独推导白线 |
| 速度调度 | 根据横向误差、航向误差、置信度和段 profile 组合 |
| 跳前对正 | 新鲜度、可见性、置信度、两类误差和连续帧共同判定 |
| 重捕获稳定确认 | 连续 N 帧通过后才从搜索转入居中或巡线 |

### 5.2 不能直接支持

`LineTrack` 当前五个字段不能直接表达：

- legacy 最大轮廓面积突增；
- legacy 多分支轮廓数量；
- legacy 内部 odometry 定距；
- legacy 内部 IMU 定角；
- `pyrealsense2` 独占相机状态；
- 白色障碍是哪一个业务事件；
- 蓝区是否为起点就绪区或最终停车区；
- 机器人物理上是否静止、离地、跨越或落地。

不得为复制 legacy 而把这些内部量塞进 `LineTrack`。需要的新证据应通过独立检测、
状态或后续 typed 接口设计。

## 6. C：目标巡线控制设计

### 6.1 启动就绪门

`START_COMMAND_LATCHED` 后仍保持零速，直到：

- `LineTrack` 接收时间新鲜；
- `line_visible=true`；
- `confidence` 达标；
- lateral 和 heading 均为有限值；
- 连续 N 帧稳定；
- command_mux、estop 和 gait lock 状态允许巡线。

超时则停在安全状态并向 mission 返回 `line_not_ready`，不得以旧消息起步。

### 6.2 速度 profile

每个巡线段由 mission 指定 profile。目标速度上限取以下约束的最小值：

- 段基础速度；
- 横向误差等级；
- 航向误差等级；
- 置信度等级；
- corner candidate 风险等级；
- 接近障碍或最终停车区的低速上限。

所有具体增益、阈值和速度均为 `NEEDS_FIELD_EVIDENCE`。

### 6.3 丢线和重捕获

目标状态：

```text
TRACK
-> SHORT_LOST_HOLD
-> TURN_LOST_KEEP 或 SEARCH_LINE
-> REACQUIRE_LINE
-> ALIGN_TO_LINE
-> TRACK
```

安全要求：

- 普通丢线不产生白线事件；
- 默认搜索不向前盲走；
- 搜索有段级总超时；
- 假重捕获必须重置连续计数；
- 从 `STOP` 恢复也必须使用稳定确认，不能单帧直接恢复；
- 重捕获只证明重新看到可信线路，重新居中必须另行完成。

## 7. D：白色障碍检测差异

### 7.1 当前检测

当前 `detect_white_bar()`：

- 将 BGR 图像转换到 HSV；
- 使用高 V、低 S 阈值得到白色 mask；
- 执行形态学清理；
- 从轮廓中选择满足最小面积、最小宽度比例和最大高度比例的横向目标；
- 通过 `SpecialTargetDetection` 发布 visible、confidence、center、面积和尺寸比例。

优点：

- 有独立 typed 检测消息；
- 有横向长条几何约束；
- 提供障碍在画面中的位置；
- 与黑线可见性分离。

缺口：

- 检测器本身没有中央下部 ROI 的场景约束；
- 没有白色行占比证据；
- 没有黑线投影间隙辅助证据；
- 业务层只有一个 `white_bar_handled`，不能表达两个语义事件；
- line-course 在任意 `LINE_FOLLOW` 阶段都可能处理白线。

### 7.2 省赛双模式

模式 1：

- 只看下三分之一的中央窗口；
- B、G、R 三通道同时高于阈值；
- 逐行统计白色像素占比；
- 要求连续白行段达到最小高度且不贴 ROI 上下边界。

模式 2：

- 对中央窗口的黑线逐行投影；
- 平滑投影后寻找位于 ROI 内部的低黑像素间隙；
- 要求间隙达到最小行高；
- 当整段黑线证据太弱时直接拒绝。

优点是加入“横向白带”和“黑线被横向物体切断”两类场景证据；风险是模式 2
仍可能把阴影、曝光或普通断线误判为白线。

## 8. E：白色障碍整合设计

分类为 `MERGE_AND_REWRITE`。

### 8.1 感知证据

保留：

- 当前 HSV 阈值；
- 当前形态学；
- 当前横向长条轮廓约束；
- 当前 `SpecialTargetDetection` 接口。

评估并新增内部证据：

- 中央下部 ROI；
- 中央 ROI 白色行占比；
- 黑线投影内部间隙。

建议候选规则：

```text
current_horizontal_white_bar
AND obstacle_position_gate
AND (central_white_row_evidence OR guarded_black_gap_evidence)
```

其中 `guarded_black_gap_evidence` 只有在间隙上下均存在可信黑线投影时才有效。
`line_visible=false`、黑线整体消失或 tracker 拒绝帧不能单独形成白色障碍证据。

### 8.2 业务阶段使能

只定义两个业务事件：

- `START_OBSTACLE`
- `END_OBSTACLE`

消费规则：

| mission 阶段 | 唯一允许事件 |
|---|---|
| `FOLLOW_TO_START_OBSTACLE` | `START_OBSTACLE` |
| `FOLLOW_TO_END_OBSTACLE` | `END_OBSTACLE` |
| 其他阶段 | 不消费；只记录被阶段门拒绝 |

不维护用于业务判断的全局 `white_count`。

### 8.3 确认、锁存和重新装填

每个语义事件分别维护：

- `enabled`：当前 mission 阶段是否允许；
- `confirm_streak`：连续有效候选数；
- `latched`：本次 RunMission 是否已消费；
- `last_detection_age`：检测新鲜度；
- `exit_clear_streak`：障碍离开画面的连续帧数；
- `cooldown_until`：冷却截止时间；
- `rearmed`：是否允许下一次物理障碍候选。

触发必须同时满足：

- 当前阶段允许；
- 检测消息新鲜；
- candidate 通过多证据；
- center_y/面积满足当前障碍 profile 的接近门限；
- 连续 N 帧成立；
- 对应语义事件未锁存；
- 不处于冷却。

事件一旦消费立即锁存。障碍仍在画面中、检测抖动或跳后再次看到同一白带都不能
重复触发。

重新装填需要：

- 已离开当前障碍；
- 连续 M 帧候选为 false；
- 冷却结束；
- 已完成跳后重捕获和重新居中；
- mission 已切换到允许下一个障碍的新阶段。

重新装填只恢复感知接收能力，不清除已完成的
`START_OBSTACLE`/`END_OBSTACLE` RunMission 锁存。

## 9. 跳跃闭环

统一流程：

```text
APPROACH_OBSTACLE
-> ALIGN_TO_LINE
-> FRONT_JUMP
-> POST_JUMP_SETTLE
-> REACQUIRE_LINE
-> ALIGN_TO_LINE
-> RESUME_TRACK
```

以上大写名称是 `[PROPOSED]` 的 mission/line-course 状态。Jump executor 内部
feedback 使用小写 `acquire_gait_lock`、`sdk_front_jump`、`post_settle` 等阶段名，
两者属于不同层级，不是同一状态枚举。终点 Jump 的上层完成状态统一命名为
`END_JUMP_SUPERVISED_DONE`，只表示受监督软件流程完成，不表示物理越障完成。

### 9.1 跳前对正

`ALIGN_TO_LINE` 放行条件必须全部满足：

- `LineTrack` 接收时间新鲜；
- `line_visible=true`；
- `confidence >= align_min_confidence`；
- `abs(lateral_error) <= align_max_lateral_error`；
- `abs(heading_error) <= align_max_heading_error`；
- 连续 N 帧成立。

任何非有限值立即失败。连续计数在一项不满足时归零。对正超时或丢线无法恢复时：

- 发布并保持零速；
- 向调用者返回失败原因；
- 禁止进入 `FRONT_JUMP`；
- 禁止沿用省赛“超时后直接跳”。

### 9.2 Jump 结果边界

FrontJump 必须通过现有 `/locomotion/execute_motion` 和 SDK2 helper 监督执行。
SDK 返回 0 只能记录：

- `sdk_command_accepted`；或
- helper 进程正常完成。

不得记录为跳跃、跨越或落地物理成功。完成 post-settle 且没有可靠姿态反馈时，
结构化日志以及现有 `ExecuteMotion.Result.message` 的文本语义必须包含：

```text
sdk_command_accepted
post_settle_completed
physical_crossing_unverified
```

`[CURRENT]` `is_robot_stable()` 仅以 TODO 占位并直接返回 True，没有读取真实且
新鲜的 roll、pitch 或角速度，因此不是有效物理稳定证据。除非后续
`[NEEDS_FIELD_EVIDENCE]` 的姿态数据链路、阈值和实机验证全部成立，否则不得用它
产生稳定、落地或跨越成功结论。

这些名称不是当前 `ExecuteMotion.action` 字段。当前 Action 仍只使用
goal `motion_name`、result `success/message`、feedback
`current_step/progress`；完整 mission 后续可把这些文本/日志语义派生为
`[PROPOSED]` RunMission 内部记录。

### 9.3 跳后统一恢复

`REACQUIRE_LINE`：

- 以零前进速度搜索；
- 要求新鲜、可见、置信度达标；
- 连续 N 帧确认；
- 超时安全失败。

`ALIGN_TO_LINE`：

- 在已重捕获基础上同时收敛 lateral 和 heading；
- 连续稳定 N 帧；
- 失败不得直接恢复巡线。

只有两步都成功后，mission 才允许 `RESUME_TRACK`。

## 10. 状态机职责

### 10.1 完整 mission

负责：

- 当前完整赛道阶段；
- 将视觉候选解释为 `START_OBSTACLE` 或 `END_OBSTACLE`；
- 决定是否允许消费障碍事件；
- 启动对应 Jump Action；
- 接收受监督执行结果；
- 决定失败、有限重试或停止；
- 保存目标放置平台和最终停车使能；
- 新 RunMission/reset 时清空所有运行锁存。

### 10.2 line-course

负责：

- 执行 mission 指定的巡线段；
- 接近当前指定障碍；
- 跳前对正；
- 跳后重捕获；
- 跳后二次对正；
- 只向 `/control/mission_cmd` 输出候选速度。

禁止：

- 全局响应任意白线；
- 全局响应任意蓝区；
- 通过“第几次看到”决定业务语义；
- 自行决定完整路线下一阶段；
- 绕过 command_mux。

### 10.3 后续 typed 分段巡线接口

建议 Action 名：

```text
/navigation/follow_line_segment
```

建议 goal：

- `segment_name`
- `terminal_event`
- `speed_profile`
- `timeout`

建议 feedback：

- `navigation_state`
- `line_visible`
- `line_confidence`
- `lateral_error`
- `heading_error`

建议 result：

- `success`
- `terminal_event`
- `failure_reason`
- `message`

该接口只在本文设计，不在本轮创建 `.action` 文件，也不替换现有
`/locomotion/execute_motion`。

## 11. 抓取平台目标锁存设计

### 11.1 当前事实

当前 `SignDetection` 只有：

- `header`
- `sign_type`
- `sign_value`
- `confidence`

当前 mission 已有 `place_platform_1/2` 映射和 `place_target` 上下文，但：

- 单帧命中即可写入；
- 检测失败时允许使用 `default_place_target`；
- 后续导航和放置仍可回退默认平台；
- 消息没有 `center_error`；
- 没有 typed 的锁存状态。

因此不能把当前骨架描述为已满足赛规。

### 11.2 后续感知输出设计

复用或扩展 `SignDetectionArray` 体系，逻辑输出至少包含：

- `marker_visible`
- `marker_id`
- `confidence`
- `target_platform`
- `center_error`

目标平台只允许：

- `place_platform_1`
- `place_platform_2`

`center_error` 是后续平台对准输入，不应塞进当前警示标识识别算法。是否扩展现有
消息或新增 typed 检测消息，应在独立接口 PR 决定；本轮不修改 schema。

### 11.3 连续确认和 RunMission 锁存

候选必须满足：

- 当前阶段为 `DETECT_PICK_SIGN`；
- 消息新鲜；
- `marker_visible=true`；
- `marker_id` 为 1 或 2；
- confidence 达标；
- 连续 N 帧为同一 marker；
- 冲突 marker 使确认 streak 归零并记录。

确认后写入活动 RunMission 上下文：

```text
run_id
target_platform
target_marker_id
target_confidence
target_latched=true
target_latched_at
placement_completed=false
```

锁存后：

- 后续同值识别只记录；
- 后续冲突识别不得改写；
- 识别失败不得选择默认平台；
- 未锁存目标时不得进入放置平台分支；
- 完成放置后可将 `placement_completed=true`，但保留目标值用于本次任务审计；
- 节点重启、新 RunMission goal 或明确 reset 必须清空整个上下文。

检测平台三个警示标识继续使用现有算法和动作映射，不因该设计而修改。

## 12. 候选逻辑分类总表

### 12.1 KEEP_CURRENT

- 当前多扫描带选线；
- 当前 route lock 和稳定候选切换；
- 当前 `LineTrack` 接口；
- 横向误差加航向误差控制；
- suggested-command 架构；
- command_mux 唯一最终速度控制；
- 命令和感知消息超时保护；
- NaN/Inf 拒绝；
- 当前搜索状态中已存在的连续重捕获确认；
- gait lock 释放时 command_mux 使旧缓存失效的语义。

### 12.2 PORT_FROM_PROVINCIAL

以下只迁移动作语义和经验，必须在 ROS2 架构内重写：

- `FrontJump` 动作语义；
- 跳跃前先停稳；
- 动作结束后的稳定等待；
- 连续稳定确认思想；
- 起点和终点使用独立参数的实机经验。

### 12.3 MERGE_AND_REWRITE

- 白线/白色横杆多证据检测；
- 启动就绪保护；
- 速度 profile 与危险弯角减速；
- 跳前对正；
- 跳后重新找线；
- 跳后二次居中；
- 障碍事件锁存；
- 离开障碍后的重新装填；
- 起点蓝区和终点蓝区的阶段化处理；
- 从 STOP 恢复时的稳定重捕获；
- 抓取平台目标连续确认和 RunMission 锁存。

### 12.4 REJECT

- 运行或安装 legacy；
- 复制 `IntegratedMission`；
- 用最大轮廓替换当前 tracker；
- legacy 直接调用 `SportClient.Move()` 参与正式比赛；
- legacy 直接成为最终速度控制者；
- 绕过 command_mux；
- 修改 Unitree UDP 协议；
- 普通丢线后盲目前进；
- 把丢线直接视为白线；
- 对正超时或丢线后强制跳跃；
- 固定时间冒充到达、重捕获或动作成功；
- SDK 返回 0 冒充物理跨越成功；
- 全局 `white_count` 决定起点和终点业务语义；
- 识别失败时默认选择放置平台；
- 修改检测平台三个警示标识算法。

### 12.5 NEEDS_FIELD_EVIDENCE

- PID I/D 项是否需要；
- 所有巡线增益、速度和阈值；
- lateral/heading 联合速度调度曲线；
- 危险弯角和大黑块旁的触发阈值；
- 横移 `vy` 对正；
- 跳前冲速度和持续时间；
- 起点和终点障碍的独立参数；
- FrontJump 真实成功率；
- 落地稳定等待时间；
- IMU/姿态稳定阈值；
- 白色障碍中央 ROI 和行占比阈值；
- 障碍离开与重新装填帧数；
- 最终停车蓝区安全内边界；
- 四个足端全部进入蓝区的实机判据。

## 13. 第一个实际开发 PR

标题：

```text
feat(locomotion): supervise start and finish FrontJump execution
```

精确范围：

- 只修改 gait action 执行、executor 自有参数、定向单元测试和必要测试辅助代码；
- 保留 `/locomotion/execute_motion` 与现有 `ExecuteMotion.action`；
- 将 `start_jump`、`finish_jump` 映射到共用 FRONT_JUMP executor；
- 使用现有 SDK2 `go2_sdk_motion_action <interface> front_jump 0`；
- 管理 `/gait/control_lock`；
- 持续向 `/control/locomotion_cmd` 发布零 Twist；
- 等待 `/navigation/cmd_vel` 连续归零；
- 执行 post-settle；
- 只在存在真实新鲜姿态数据时启用可选稳定确认；
- 所有退出路径先保持零速再释放 lock；
- 起点和终点 executor 参数独立。

明确不修改：

- line tracker；
- line follower；
- 白色障碍 detector；
- 完整 mission；
- 抓取平台识别；
- 检测平台三个警示标识算法；
- 迷宫、楼梯、机械臂；
- UDP 协议；
- legacy；
- Action schema。

详细监督顺序、结果语义和验证要求见
`docs/JUMP_VALIDATION_PLAN.md`。
