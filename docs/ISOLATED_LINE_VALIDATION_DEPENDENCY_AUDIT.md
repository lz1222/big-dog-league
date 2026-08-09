# Isolated line validation dependency audit

本审计限定于 `competition_non_arm.launch.py` 的
`start_mission_nodes=false`、`readiness_profile=isolated_line_validation`
组合。它记录被有意移除的 provider 及剩余图的依赖，不改变生产算法、优先级
或控制参数。

## 启动图

继续启动的 ROS 节点为：

- `line_camera_node`
- `real_line_tracker_node`（同时提供 LineTrack 与 white-bar 感知）
- `real_sign_detector_node`
- `line_follower_node`
- `gait_control_node`
- `command_mux_node`
- `gait_lock_arbiter_node`
- `go2_front_camera_bridge_node`
- `competition_readiness_node`
- `cmd_vel_udp_forwarder`

硬件模式还启动经 runtime wrapper 隔离的 `go2_sdk_udp_server` 进程。

由 `start_mission_nodes=false` 明确禁用的节点为：

- `line_course_mission_node`
- `white_bar_stage_command_publisher`
- `white_bar_action_executor`
- `inspection_action_executor`

## Disabled-provider dependency matrix

| 被移除的 provider / 接口 | 剩余 consumer | 既有行为 | isolated 契约 |
|---|---|---|---|
| `inspection_action_executor` → `/gait/control_lock_req/inspection` | `gait_lock_arbiter_node` | 未见心跳永久锁定 | inspection 显式 `required=false`；保留真实 `seen/fresh/value`，不得伪造消息 |
| mission/action 状态 topics 与 `/locomotion/execute_motion` | `competition_readiness_node` | production 必须可用/idle | 仅既有 allowlist 项标记 `SKIPPED_BY_PROFILE:isolated_line_validation` |
| `line_course_mission_node` → `/control/mission_cmd` | `command_mux_node` | 无 fresh candidate 时不参与选择 | 保持订阅但不要求 provider；动态门另验 publisher=0、selected source=line |
| mission route → `/mission/start` | `line_follower_node` | 正式状态机启动输入 | 仅 follower 改接 `/validation/line_follow/start`；mission/action 节点不接收该 topic |
| mission route → `/mission/stop` | `line_follower_node` | 可选异步停车事件 | 缺失不构成 liveness 条件；所有真实安全停车条件保持有效 |
| white-bar / inspection action status | `mission_stop.sh` cleanup | production 等待 terminal 证明 | 启动器冻结 profile/图合同；isolated 在 estop 与连续 mux-zero 后显式标记 skipped，不等待不存在的 provider |

其余保留节点没有对 `line_course_state`、white-bar action status、inspection
action status 或 inspection lock heartbeat 建立额外 service/action wait、watchdog、
状态机门或 liveness 门。`gait_control_node` 仍是必需 provider：其 gait lock 请求
必须 `seen=true`、`fresh=true`、`value=false`，否则 isolated 同样 fail-closed。

## Profile consistency

- `production` 只允许 `start_mission_nodes=true`，并要求 gait、inspection 两路。
- `isolated_line_validation` 只允许 `start_mission_nodes=false`，仅跳过 inspection。
- profile、graph flag 与 input required-set 都是启动期参数；运行时修改被拒绝。
- arbiter 状态逐来源输出 `required/seen/fresh/value/age_sec/reason`。
- readiness 同时验证实际 `/gait/control_lock=false`、arbiter profile、图一致性及
  required-set，避免“readiness PASS 但 follower 因锁输出零”的假闭环。
- cleanup 从 runtime 读取同一冻结合同；未知、缺失或混搭值一律按 production
  要求 action terminal 证明，只有精确 isolated/false 组合允许跳过。

静态搜索范围包括 `rk_bringup`、`rk_navigation`、`rk_safety`、`rk_locomotion`
中的 publisher/subscription、service/action client、heartbeat、fresh/stale、lock
和 mission/action 状态引用。生产 profile 默认值与全部生产必需来源保持不变。
