# SDK_BRIDGE_RELIABILITY_V2 — 审计与重构

## F1: Watchdog 审计

### 系统中共3个Watchdog

| # | 名称 | 进程 | 监控对象 | Timeout | 周期 | 触发动作 | 日志标识 |
|---|------|------|---------|---------|------|---------|---------|
| 1 | **UDP_RX_TIMEOUT** | `go2_sdk_udp_server` (C++) | UDP数据包间隔 | 0.30s | Tick @20Hz | `ForceStop("watchdog_timeout")` → `StopMove()` | `watchdog_timeout` |
| 2 | **FORWARDER_TIMEOUT** | `cmd_vel_udp_forwarder.py` (Python) | ROS2 Twist接收间隔 | 0.30s | Timer @20Hz | `send_stop_once("forwarder_watchdog")` → UDP零包 | `forwarder_watchdog` |
| 3 | **COMMAND_SOURCE_TIMEOUT** | `command_mux_node` (Python) | line/mission/locomotion命令间隔 | 0.5/0.5/0.3s | Evaluate @20Hz | 切换到零Twist | `no_fresh_command` |

### 当前真正触发Stop的Watchdog

**FORWARDER_TIMEOUT** (Watchdog #2) 是导致频繁StopMove的根因。

原因: `maze_full_auto.py` 的感知循环中 `rclpy.spin_once()` 阻塞时间 + 处理时间合计超过0.3s，
导致Forwarder收不到新Twist，触发watchdog发送UDP零包给SDK Server。

日志证据: 每约2秒出现一次 `[UDP] send vx=0.000 ... reason=forwarder_watchdog`

### SDK Server挂掉根因

SDK Server (`go2_sdk_udp_server`) 在以下情况death:
1. `STARTUP_STOPMOVE_RETRY_EXHAUSTED` — Go2 Sport DDS channel未就绪 (需要SLAM激活)
2. 进程被pkill / 终端关闭

### 分层解决方案

V2架构:
```
[Planner] → /control/locomotion_cmd → [command_mux] → /navigation/cmd_vel
→ [V2 Forwarder: 持续发布, seq追踪] → UDP (protocol_v2)
→ [V2 SDK Server: Thread A收包 + Thread B运动 + Thread C健康]
→ SportClient.Move()
```

## F2: 端到端追踪 (V2协议)

```cpp
// UDP Protocol v2
struct UdpPacket {
    int protocol_version;  // 2
    uint64_t session_id;   // 每次SDK重启递增
    uint64_t seq;          // 每包递增
    int64_t monotonic_ns;  // 发布时CLOCK_MONOTONIC
    double vx, vy, wz;     // 速度命令
    int flags;             // bit0=boost, bit1=estop
};
```

## F3: SDK Server多线程架构

```
Thread A (UDP Receiver):  ~无限循环 recvfrom()
  → 验证session/seq/nan
  → 更新 SharedState.latest

Thread B (SDK Motion Loop):  50Hz
  → 读取 SharedState.latest
  → 检查 freshness (UDP_RX watchdog)
  → 调用 SportClient.Move() 或 StopMove()
  → 记录调用结果

Thread C (Health Monitor):  每5秒
  → 输出诊断: packets, drops, sdk_calls, sdk_failures
```

## F4: 分层Watchdog

| Watchdog | 层次 | 默认Timeout | 测量方法 |
|----------|------|------------|---------|
| WATCHDOG_UDP_RX | SDK Server | 0.30s | packet_age > t → StopMove |
| WATCHDOG_SDK_HEALTH | SDK Server | 连续3次失败 | consecutive_failures > 3 → WARN |
| WATCHDOG_CMD_AGE | Forwarder | 0.30s | last_cmd_age > t → UDP零包 |
| WATCHDOG_MOTION_FEEDBACK | Planner | Odom反馈 | cmd非零但Odom静止 → STOP |

## 下一步

完成上述V2 C++文件的SportClient集成后，执行R1-R3真机测试。
