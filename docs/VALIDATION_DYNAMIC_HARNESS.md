# Validation dynamic harness

动态验收把实时安全控制与证据持久化拆成两个独立进程。该结构只属于
validation harness，不改变 production line follower、PID、感知、mux、gait lock
或 SDK 行为。

## Processes

`validation_motion_adapter` 只订阅 latest LineTrack、follower candidate、gait lock、
mux 状态和当前 SDK status，输出 `/control/line_cmd`。启动后保持 pre-arm 沉默；
操作员通过 `/validation/motion_adapter/arm` 的 `SetBool(true)` 才能发出一次性动态
窗口。callback 只更新 latest-state，timer 不执行文件 I/O。

`validation_dynamic_recorder` 订阅完整证据链，callback 使用 `put_nowait` 写入有界
队列。独立 writer thread 按 batch 或 100 ms 周期写 JSONL；关闭时 drain、flush、
`fsync`。队列满会增加 `dropped` 并写入 `RECORDER_BACKPRESSURE`，绝不阻塞 adapter。

## Freshness and timer safety

- 实时 freshness 使用 adapter 进程自己的 monotonic receive timestamp。
- ROS source header 与 `source_sequence` 仅用于离线区分 source stall 和 observer
  stall，禁止直接与 steady clock 相减。
- LineTrack 和控制候选使用小深度 latest-state QoS，避免恢复后先消费长队列旧帧。
- timer lateness 超过预算时以 `ADAPTER_EXECUTOR_STALL` 独立 fail-closed，不能报告
  为 `LINE_SOURCE_STALE`。
- recorder writer stall/backpressure 不得改变 adapter timer 或 `/control/line_cmd`。

## Stop reasons

明确的主要停车原因包括 `LINE_SOURCE_STALE`、`SUGGESTED_CMD_STALE`、
`GAIT_LOCKED`、`SDK_ERROR`、`MUX_SOURCE_INVALID`、
`ADAPTER_EXECUTOR_STALL`、`MOTION_WINDOW_COMPLETE` 和 `OPERATOR_STOP`。

## Zero-motion usage

仅启动节点不会 arm，也不会向 `/control/line_cmd` 发布零或非零命令。零运动集成
可在 follower candidate 与感知正常时给 recorder 注入 `--writer-delay-sec 0.3`，
验证 adapter status heartbeat 正常且 SDK `MOVE` 计数仍为零。只有完成全部静态门
并获得现场授权后才允许调用 arm service。
