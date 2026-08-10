# CLASSIC LINE + CORNER ACCEPTANCE

本运行单只覆盖经典步态下的直线巡线和单段 90° 左/右弯。红圆、白横线续航、inspection、机械臂、maze 与完整任务均不在本轮范围内；不得修改 tracker 中心偏置 `0.0422` 或正式 follower 参数。

## 已冻结的直线基线

`vx=0.27`、`kp_lateral=0.85`、`kp_heading=0.35`、`max_angular_z=0.28`、`angular_deadband=0.08`、`angular_smoothing_alpha=0.22`。全部横向速度必须为零。

每次实体运行前只读取并保存 `CheckMode` 与 `sportmodestate`；正式测试链不得调用 sport mode OFF/ON 或 `ReleaseMode`，也不得因 mcf 事件自动恢复步态。

## P0 顺序

1. 禁用 `/mission/start`，直接给最终运动链一次 `vx=0.27, vy=0, yaw=0` 的 1.5 秒开环命令；随后立刻零速和 `StopMove`。不得让 `LineTrack` 参与。明显左前侧身、非直走或姿态异常记为 `CLASSIC_OPEN_LOOP_BODY_DRIFT` 并停止；否则记 `CLASSIC_OPEN_LOOP_PASS`。
2. 保持经典步态，接通 USB line camera → `real_line_tracker_node` → `line_follower_node` → mission/mux → SDK，只运行 3 秒。保存 `/perception/line_track`、`/navigation/line_follow_status`、`/mission/line_course_state`、`/navigation/cmd_vel` 与 `/go2/sdk_motion_status`。仅在没有持续单侧发散、侧身或大幅跨线，且 `vy` 始终为零时记 `CLASSIC_STRAIGHT_3S_PASS`。
3. 不修改任何变量，仅在步骤 2 通过后运行 5 秒。通过记 `CLASSIC_STRAIGHT_5S_PASS`；此时直线控制基线冻结，不直接重复 10 秒。

`/navigation/line_follow_status` 的 `raw_angular_z`、`target_angular_z`、`smoothed_angular_z` 分别是 PID 原始值、限幅/死区目标和滤波输出。`/mission/line_course_state` 的 `mux_final_vx`、`mux_final_vy`、`mux_final_yaw` 才是 mux 的最终命令；不要把历史兼容字段 `final_vx/final_wz` 当作最终命令。

## P1 角点合同与顺序

正式 owner 固定为 `line_course_mission_node`。当候选确认时，mission 进入 `CORNER_PRE_TURN` 并发布唯一的转弯候选；follower 丢线只能进入 `CORNER_OWNER_WAIT` 并持续发布零，不会进入 `TURN_LOST_KEEP`、`TURN_90` 或 `SEARCH_LINE`。

状态时序为：

`LINE_FOLLOW` → `corner_candidate` 连续确认 → `CORNER_PRE_TURN`（mission） + `CORNER_OWNER_WAIT`（follower 零输出） → `ALIGN_TO_LINE`（mission 原地对正） → 连续新线确认 → `LINE_FOLLOW`。

方向配置显式为 `corner_turn_direction: hint`。只有 detector 输出 `left` 或 `right` 才会触发；`unknown` 会以 `corner_direction_unknown` 零速拒绝，绝不默认左转。实体转弯前，静态分别采集左/右弯的接近与触发位置，以及直线对照，保存 `/perception/line_track`、`/perception/corner_candidate` 和 debug overlay。静态证据必须证明左右 hint 可区分且直线不误触发，才能开始真机转弯。

静态审计时可仅以 launch 参数覆盖 `real_line_tracker_node.enable_debug_image:=true` 来取得 overlay；不要改动检测阈值、中心偏置或任何速度/PID。采集结束恢复该调试开关即可。

静态通过后只按以下顺序运行：单段左弯（进入、触发、转弯、重获新线、稳定约 1 秒、立即停止）→ 单段右弯。每段保存首次检测时间与置信度、`direction_hint`、触发前 line 误差、实际 mux `vx/vy/yaw`、最大转向时长、丢线/重获时刻、重获帧数、`ALIGN_TO_LINE` 时长和最终 line 误差。

失败先分类为 `PERCEPTION_TRIGGER_EARLY`、`PERCEPTION_TRIGGER_LATE`、`WRONG_DIRECTION`、`DOUBLE_TURN_OWNER`、`INSUFFICIENT_PRETURN`、`OVERSHOOT`、`LINE_REACQUIRE_FAIL` 或 `ALIGN_FAIL`；一次只改对应层。

最终结果只能依次记为：`CLASSIC_OPEN_LOOP_PASS`、`CLASSIC_STRAIGHT_3S_PASS`、`CLASSIC_STRAIGHT_5S_PASS`、`CORNER_STATIC_PASS`、`CLASSIC_LEFT_CORNER_PASS`、`CLASSIC_RIGHT_CORNER_PASS`、`CLASSIC_CORNER_LINE_REACQUIRE_PASS`。在全部完成前，不得标记 `CLASSIC_LINE_AND_CORNER_PASS`。

报告格式：

```text
CLASSIC LINE ACCEPTANCE
OPEN LOOP: <PASS / evidence / blocker>
STRAIGHT 3S: <PASS / evidence / blocker>
STRAIGHT 5S: <PASS / evidence / blocker>
CONTROL PARAMS CHANGED: NO
CORNER OWNER: line_course_mission_node
CORNER DIRECTION SOURCE: corner_candidate.direction_hint
STATIC LEFT: <result>
STATIC RIGHT: <result>
LEFT PHYSICAL: <result>
RIGHT PHYSICAL: <result>
REACQUIRE: <result>
RESULT: <CLASSIC_LINE_AND_CORNER_PASS or concrete blocker>
```
