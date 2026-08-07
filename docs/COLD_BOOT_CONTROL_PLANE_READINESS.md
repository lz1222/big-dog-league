# Go2 冷启动控制面就绪

## 根因优先级

三次 `startup StopMove ret=-1` 均发生在机器狗重新开机后验证。因此首要
假设是：网卡和相机可能已可用，但 Go2 的 Sport/DDS 控制服务尚未完成启动。
`ping` 成功不等于可以调用 SportClient。

动态库隔离仍是必要前置条件，但它不能取代此门禁。失败分类的优先级为：

1. `ROBOT_NETWORK_NOT_READY`：接口、路由或连续 ping 尚未稳定；
2. `ROBOT_DDS_NOT_DISCOVERED`：没有收到 `rt/sportmodestate`；
3. `ROBOT_STATE_STREAM_FORMAT_INVALID` / `ROBOT_STATE_STREAM_NOT_FRESH`：
   收到的帧格式错误或不连续；
4. `ROBOT_CONTROL_PLANE_NOT_READY`：门禁或 SDK listening 阶段未放行；
5. `STARTUP_STOPMOVE_RETRY_EXHAUSTED`、`SDK_RUNTIME_LIBRARY_ERROR`、
   `UDP_BIND_ERROR`、`ROS_GRAPH_READINESS_ERROR`：后续阶段故障。

## 现有证据与缺口

仓库仅有一次历史成功记录：
`log/ros/2026-08-02-23-00-54-257784-ubuntu-20299/launch.log` 中的
`StopMove reason=startup ret=0`。该记录没有以下时间点，故不能据此倒推出
帧数或时窗：

| 时间线字段 | 历史状态 |
| --- | --- |
| `ROBOT_POWER_ON` | 未记录 |
| `FIRST_LINK_UP` / `FIRST_PING` | 未记录 |
| `FIRST_CAMERA_FRAME` | 未记录 |
| `FIRST_DDS_STATE` / `FIRST_STABLE_DDS_STATE` | 未记录 |
| `FIRST_STOPMOVE_ATTEMPT` / `FIRST_STOPMOVE_SUCCESS` | 仅成功 StopMove 一行，无相对开机时间 |

下一次冷启动必须保留 `control_plane_gate.log` 和 `sdk_server.log`；前者会记录
`ROBOT_POWER_ON`（操作者提供的锚点）、链路、首 ping、probe 执行和稳定 DDS
时间，后者记录每次 StopMove 的返回值、耗时及 UDP listening。相机首次帧尚未
由当前启动器采集，需从相机节点日志或采集脚本同步标注，不能事后猜测。

## 生产门禁

`go2_control_plane_gate.py` 只读取 `ip`、路由、ICMP 和
`go2_sdk_sport_state_monitor --gate`。monitor 只执行
`ChannelFactory::Init(0, eth0)` 并订阅 `rt/sportmodestate`；它不链接或创建
`SportClient`，不调用任何动作接口。

为避免伪造“合理默认值”，正式入口拒绝未配置的测量阈值。完成至少多次冷启动
测量后，向实际使用的入口导出以下变量：

```bash
# start_non_arm_competition.sh 使用 RK_COMPETITION_ 前缀。
export RK_COMPETITION_CONTROL_PLANE_NETWORK_TIMEOUT_SEC=<实测最大等待上限>
export RK_COMPETITION_CONTROL_PLANE_PING_COUNT=<实测连续 ping 帧数>
export RK_COMPETITION_CONTROL_PLANE_PING_POLL_SEC=<采样周期>
export RK_COMPETITION_CONTROL_PLANE_DDS_TIMEOUT_SEC=<实测 DDS 等待上限>
export RK_COMPETITION_CONTROL_PLANE_REQUIRED_FRAMES=<实测连续状态帧数>
export RK_COMPETITION_CONTROL_PLANE_MAX_FRAME_GAP_MS=<实测最大允许间隔>
```

`start_line_system.sh` 使用同名但不含 `RK_COMPETITION_` 的变量。门禁成功后才
启动 UDP server；server 的 `startup StopMove` 最多尝试三次、记录每次耗时，
成功并打印 `UDP server listening on` 后才启动相机、转发器和其余 ROS 图。任一
阶段失败均不会进入 UDP listening 或 readiness 成功状态。
