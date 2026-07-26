# 比赛控制权与速度仲裁

## 适用范围

本文描述 `codex/national-control-architecture` 分支、基线提交
`ded2631` 之上的当前控制权实现。它只覆盖“谁可以把候选速度送到最终
`/navigation/cmd_vel`”这一层，不证明完整比赛状态机、Unitree 硬件动作或
170 分任务已经完成。

当前 `competition_line_nav.launch.py` 与 `start_line_system.sh` 的真实巡线链
使用 `command_mux_node` 作为最终速度唯一发布者。mock、视觉调试、避障调试、
警示动作调试和工具节点仍保留独立直发路径；它们不得与真实比赛控制栈并行。

## 真实巡线控制数据流

```text
D435i
  -> real_line_tracker_node
  -> /perception/line_track 及特殊目标 topics
  -> line_follower_node
  -> /navigation/line_follow_cmd_suggested
  -> line_course_mission_node
  -> /control/mission_cmd
  -> command_mux_node
  -> /navigation/cmd_vel
  -> cmd_vel_udp_forwarder
  -> UDP 127.0.0.1:15001
  -> 仓库外 go2_sdk_udp_server
  -> Unitree Go2
```

`line_follower_node` 本轮仍发布 suggested command，不直接接入 mux。
`/control/line_cmd` 是为后续架构升级保留的输入；当前真实巡线 launch 中没有
该 topic 的业务发布者。`gait_control_node` 的默认速度输出已是
`/control/locomotion_cmd`，但真实巡线 launch 当前不启动 gait action server。

## ROS 接口

| 方向 | 默认接口 | 类型 | 当前真实巡线来源/去向 |
| --- | --- | --- | --- |
| 输入 | `/control/line_cmd` | `geometry_msgs/msg/Twist` | 预留，当前无业务发布者 |
| 输入 | `/control/mission_cmd` | `geometry_msgs/msg/Twist` | `line_course_mission_node` |
| 输入 | `/control/locomotion_cmd` | `geometry_msgs/msg/Twist` | gait 默认输出；当前 launch 未启动 gait |
| 输入 | `/safety/estop` topic | `std_msgs/msg/Bool` | 持续/自动急停输入 |
| 输入 | `/safety/estop` service | `std_srvs/srv/SetBool` | 运维脚本和人工急停入口；由 mux 自身提供 |
| 输入 | `/gait/control_lock` | `std_msgs/msg/Bool` | gait server 启动时发布 |
| 输入 | `/arm/control_lock` | `std_msgs/msg/Bool` | arm server 启动时发布 |
| 输出 | `/navigation/cmd_vel` | `geometry_msgs/msg/Twist` | mux 的最终速度输出 |
| 输出 | `/control/cmd_mux_status` | `std_msgs/msg/String` | 紧凑 JSON 仲裁状态 |

所有接口都可通过 `command_mux_node` 参数覆盖。正式比赛组合应保持上表默认
控制契约，不能通过 remap 绕过 mux 重新形成多个最终发布者。

## 仲裁优先级

优先级固定如下：

1. `estop=true`：无条件零速度，`active_source=estop`。
2. `arm_lock=true`：无条件零速度，`active_source=arm_lock`。
3. `gait_lock=true`：仅允许新鲜的 locomotion command；无命令或超时均输出
   零速度，且不得回退到 mission/line。
4. 无 lock 时，新鲜 mission command 优先。
5. mission 不新鲜时，使用新鲜 line command。
6. 没有新鲜合法输入时输出零速度。

普通模式不会直接使用 locomotion command；locomotion 只有在
`gait_lock=true` 时才可获得控制权。

## 默认频率、超时和限值

| 参数 | 默认值 |
| --- | ---: |
| `control_rate_hz` | `20.0 Hz` |
| `line_cmd_timeout_sec` | `0.5 s` |
| `mission_cmd_timeout_sec` | `0.5 s` |
| `locomotion_cmd_timeout_sec` | `0.3 s` |
| `max_linear_x` | `0.60 m/s` |
| `max_linear_y` | `0.15 m/s` |
| `max_angular_z` | `1.30 rad/s` |

超时和限值必须是大于零的有限数，非法配置会让节点启动失败。这些数值是 mux
安全上限，不是巡线目标速度；本轮没有修改巡线 PID 或实际速度参数。

合法但超限的输入会在 mux 内按正负对称限幅，状态中的 `clamped` 会置为
`true`。任一速度分量为 NaN/Inf，或命令结构无法转换为数值时，整条命令会被
拒绝、所有候选缓存失效、`invalid_command_count` 增加，并至少输出一个零速
周期。mux 不修改收到的 ROS message 对象。

下游 `cmd_vel_udp_forwarder` 仍会执行第二次 finite 检查、deadband 和自身
`max_vx/max_vy/max_yaw` 限幅。实际发往 UDP 的值因此可能比 mux status 中的
`final_vx/final_vy/final_wz` 更小；应同时检查 mux 状态和 forwarder 参数。

## 锁解除、旧命令和时间语义

- `estop` 在 `false -> true` 和 `true -> false` 两种真实状态转换时，都清空
  三个来源的命令、接收时间、有效性、新鲜度和 clamp 状态。
- `arm_lock` 或 `gait_lock` 从 `true` 变为 `false` 时，三个来源的旧命令失效。
- 重复接收与当前值相同的 estop true/false 是幂等操作，不再次清除状态变化后
  新收到的缓存。
- 解除后必须收到一条新的、合法且符合当前仲裁条件的命令，机器人才能恢复
  移动。
- ROS 时间倒退时，已有命令全部失效并输出零速度；恢复后的旧缓存不会自动
  生效。
- gait lock 下 locomotion 超时后保持零速度，不回退到 mission/line。

这些规则阻止“解除急停/锁后旧命令突然恢复”，但不能替代机器人端制动距离、
SDK watchdog 或硬件急停验证。

## 急停软件闭环

真实巡线组合中的 `command_mux_node` 同时提供同名 service 并订阅同名 topic：

```text
/safety/estop service  std_srvs/srv/SetBool
/safety/estop topic    std_msgs/msg/Bool
```

`competition_line_nav.launch.py` 和 `start_line_system.sh` 显式启用
`enable_estop_service=true`，并把 `estop_service_name`、`estop_topic` 都配置为
`/safety/estop`。service 与 topic 虽是两个 ROS graph 接口，但在 mux 内使用
同一状态转换语义：

- `true` 进入 `estop`，mux 继续作为唯一最终发布者并周期输出零速度；
- `false` 解除 `estop`；
- true/false 的真实转换都会清空三个候选的命令、时间戳和有效性；解除后必须
  收到新命令才能恢复运动；
- 重复 true/false 不清缓存，service response 会明确报告 already/unchanged；
- service response 只表示 mux 接受并应用了软件状态，不是 UDP 或硬件停止 ACK。

旧 `rk_tools/safety_node` 也提供同名 `SetBool` service，但它属于 mock
standalone 路径，不能与真实 mux service 同时启动。真实比赛/巡线组合的 service
owner 必须是 `command_mux_node`。

该 service -> mux -> final zero 的软件闭环已在 VM/Humble 隔离 ROS 测试中验证；
机器人/Foxy 上的 service 可用性、DDS 时延、UDP payload、SDK 接收和实体停止
距离仍未验证。

### 正常停机与 emergency fallback

`stop_line_system.sh` 的正常停机路径保持最终速度唯一所有者不变：

1. 检查 `/safety/estop` service 类型为 `std_srvs/srv/SetBool`。
2. 调用 `{data: true}`，要求返回 `success=true`。
3. 从 `/navigation/cmd_vel` 抽样确认 mux 正在发布全零 Twist。
4. 只有确认零速后，才发送 `/mission/stop` 并停止后台进程。

若 service 调用成功但未观测到 mux 零速，脚本报错退出并保留节点运行，避免在
确认停止前删除唯一速度所有者；此情况不会自动进入 fallback。

只有 service 不存在、类型不符、调用失败或没有返回 `success=true` 时，
`EMERGENCY FALLBACK` 才直接向 `/navigation/cmd_vel` 发布一次零 Twist，然后
继续停机。这是服务失效时的最后防线，会短时绕过正常 mux 唯一发布者架构，
不属于正常运行的唯一发布者保证，也没有获得 UDP/机器人停止 ACK。

## 状态 JSON

`/control/cmd_mux_status` 每个控制周期发布可由 `json.loads` 解析的紧凑 JSON，
至少包含：

```json
{
  "active_source": "mission",
  "reason": "fresh_mission_command",
  "estop": false,
  "arm_lock": false,
  "gait_lock": false,
  "line_fresh": false,
  "mission_fresh": true,
  "locomotion_fresh": false,
  "line_age_sec": null,
  "mission_age_sec": 0.02,
  "locomotion_age_sec": null,
  "final_vx": 0.27,
  "final_vy": 0.0,
  "final_wz": 0.0,
  "clamped": false,
  "invalid_command_count": 0
}
```

状态中的 age 会保留超时诊断信息，数值不会输出 NaN/Infinity。该 topic 只说明
mux 的软件仲裁决定，不是 UDP server 回包、Unitree SDK 返回码、机器人运动
状态或硬件 ACK。

## Standalone 直发入口

以下入口显式保留 `/navigation/cmd_vel` 直发行为，用于单项 mock/调试，不接入
比赛 mux：

| 入口/节点 | 直发来源 |
| --- | --- |
| `mock_competition.launch.py` | `line_course_mission_node` |
| `vision_nav_debug.launch.py` | `line_course_mission_node` |
| `obstacle_practical.launch.py` | gait，另选择一个 bridge |
| `sign_action_debug.launch.py` | gait |
| `obstacle_direct_open_loop.launch.py` | `obstacle_direct_route_node` |
| `keyboard_route_node` | 键盘/路线回放 |
| `cmd_vel_speed_sweep_node` | 速度扫描 |
| `two_step_walk_test_node` | 两段行走测试 |

这些入口的存在不是重复发布故障本身；同时启动其中任一个与真实比赛控制栈，
才会产生危险的最终 topic 多发布者。正式运行前必须检查 ROS graph，而不能只
依靠静态 launch 审计。

## 调试与验收命令

先确认真实链路的 topic 类型和连接：

```bash
ros2 topic info -v /navigation/line_follow_cmd_suggested
ros2 topic info -v /control/mission_cmd
ros2 topic info -v /control/locomotion_cmd
ros2 topic info -v /navigation/cmd_vel
ros2 topic info -v /control/cmd_mux_status
ros2 topic echo /control/cmd_mux_status std_msgs/msg/String
```

确认最终速度唯一发布者：

```bash
ros2 topic info -v /navigation/cmd_vel
ros2 service type /safety/estop
```

真实 `competition_line_nav` 或 `start_line_system.sh` 组合的期望结果是：

```text
Publisher count: 1
Publisher node: command_mux_node
Subscriber: cmd_vel_udp_forwarder
```

若 publisher count 不是 1，先停止所有 standalone launch、keyboard route、
speed sweep、two-step test 和一次性 `ros2 topic pub`，再重新检查。不要在
发布者不唯一时发送 `/mission/start`。

只观察候选与最终输出，不连接真实机器人发送测试非零速度：

```bash
ros2 topic hz /navigation/line_follow_cmd_suggested
ros2 topic hz /control/mission_cmd
ros2 topic hz /navigation/cmd_vel
ros2 topic echo /navigation/cmd_vel geometry_msgs/msg/Twist
```

开发隔离环境可用独立测试 topic 验证 mux；不要在真实比赛 graph 上直接发布
非零 `/control/*_cmd`。机器人现场可在安全架空/受控条件下调用：

```bash
ros2 service call /safety/estop std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /control/cmd_mux_status std_msgs/msg/String
ros2 topic echo /navigation/cmd_vel geometry_msgs/msg/Twist
```

service success 和零 Twist 仍只是 ROS 软件链证据，不能代替 UDP server、SDK 或
实体停止验收。解除急停必须由受控流程调用 `{data: false}`，随后等待新的候选
命令；旧命令不会恢复。

## 常见故障

| 现象 | 优先检查 |
| --- | --- |
| `/navigation/cmd_vel` 无 publisher | `command_mux_node` 是否启动、`rk_safety` 是否已构建/source |
| 最终 publisher 多于 1 | 是否误启 standalone launch、工具节点或遗留 CLI publisher |
| mux 持续 `active_source=none` | `/control/mission_cmd` 是否有新鲜 publisher；mission 是否已 start |
| `active_source=gait_lock_stale` | gait lock 为 true，但 locomotion 输入缺失或超过 `0.3 s` |
| `active_source=arm_lock` | arm lock 尚未解除；即使有 mission 也应保持零速 |
| `/safety/estop` service 不存在 | 确认真实 launch 启用了 mux service，且没有 mock `safety_node` 抢占同名 service |
| service 返回成功但最终 topic 不是零 | 保留 mux 运行并检查 status/ROS graph；正常 stop 脚本会报错且不走 fallback |
| stop 日志出现 `EMERGENCY FALLBACK` | service 缺失或调用失败，脚本已直发一次零 Twist；唯一发布者保证已被绕过，必须记录并排障 |
| `invalid_command_count` 增加 | 上游发送 NaN/Inf/不可转换命令；所有候选缓存会失效 |
| mux status 速度正常但机器人不动 | 检查 UDP forwarder 二次限幅/deadband、订阅、UDP server、网卡和 SDK；status 不是 ACK |
| 解锁后仍为零 | 安全设计要求解锁后收到新命令，旧缓存不会恢复 |
| 时间重置后为零 | 时间倒退会主动清除候选，等待新命令 |

## 完成度边界

command mux 的纯核心测试和 VM/Humble 隔离 ROS smoke 可以证明软件仲裁与
SetBool/topic 统一状态转换在测试环境中按预期工作；它们不能证明：

- `stairs_up_down` 已有真实实现；
- `start_jump`/`finish_jump` 已调用跳跃 SDK；
- 机械臂存在真实 subscriber、ACK 或已标定姿态；
- 白横线动作、真实 item perception 或完整比赛状态机已经接线；
- UDP server 已接收零速度、Go2 已执行或实体已经停止；
- 170 分场内任务任一项已完成硬件验收。

正式结论必须来自机器人 Foxy 环境的唯一发布者检查、service/topic 两种急停
入口联调、正常停机零速确认、SDK/UDP 日志和实体停止验收。任何
`EMERGENCY FALLBACK` 运行都必须单独记录，不能算正常唯一发布者路径通过。
