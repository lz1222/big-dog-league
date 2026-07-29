# B1.5 Go2 Deterministic UDP Motion Bridge

## 1. 目标与边界

本阶段将 `/navigation/cmd_vel` 与 Unitree SDK2 隔离到两个进程：

```text
ROS2 Foxy cmd_vel
  -> cmd_vel_udp_forwarder.py
  -> UDP 127.0.0.1:15001
  -> go2_sdk_udp_server
  -> Unitree SportClient.Move / StopMove
```

这样可以避免 ROS2 Foxy CycloneDDS 0.7 与 Unitree SDK2 CycloneDDS 0.10
在同一进程中加载。服务端源码、构建目标和安全逻辑均已纳入
`rk_go2_sdk_bridge`，B1.5 启动路径不再依赖
`/home/unitree/unitree_go2_sdk_test/build/go2_sdk_udp_server`。

本阶段只建立可审计的运动桥。B1/B2 感知与决策、巡线、
`gait_control_node` 和 command mux 均未修改。

## 2. 已确认问题

旧外部服务端使用 `step_vx=0.03` 逐级接近目标。`vx=0.25m/s` 的一秒
测试约需 0.45 秒才达到目标速度，真机出现一次仅前倾、一次移动约
25cm 的不稳定结果。停车时旧服务端还会逐级下降到约 `0.04m/s` 后
才调用 `StopMove`。

新服务端移除了该渐变：

- 新鲜非零目标以固定 20Hz 原值调用 `SportClient.Move`。
- 收到零速度立即调用一次 `StopMove`。
- 服务端超过 0.30 秒未收到有效非零命令，立即进入 `StopMove`。
- 非法、非有限、附加字段、超长和越界报文立即进入 `StopMove`。
- 启动、SIGINT、SIGTERM、SDK 错误和已初始化后的异常路径尝试
  `StopMove`。
- 默认不调用 `BalanceStand`，不改变操作员选择的步态。

进程无法捕获 `SIGKILL`、内核崩溃、断电或 SDK 固死。因此遥控急停
仍是独立的最终安全层，不能由软件 watchdog 替代。

## 3. 默认参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `sdk_network_interface` | `eth0` | Unitree SDK2 网卡 |
| `udp_host` | `127.0.0.1` | 本机 UDP 地址 |
| `udp_port` | `15001` | UDP 端口 |
| `sdk_rate_hz` | `20.0` | `Move` 固定输出频率 |
| `timeout_sec` | `0.30` | 转发器与服务端 watchdog |
| `max_vx` | `0.25` | 前后速度绝对值上限 |
| `max_vy` | `0.05` | 横向速度绝对值上限 |
| `max_yaw` | `0.60` | 角速度绝对值上限 |
| `deadband` | `0.01` | 零速死区 |

超过上限的命令不会被静默截断，而是触发停车。这样可以暴露上游错误，
避免错误控制量被伪装成正常极限速度。

## 4. 虚拟机验证

纯逻辑测试不链接 Unitree SDK，不发送 UDP，也不控制机械狗：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  -I src/rk_go2_sdk_bridge/include \
  src/rk_go2_sdk_bridge/src/udp_motion_core.cpp \
  src/rk_go2_sdk_bridge/test/test_udp_motion_core.cpp \
  -o /tmp/b15_test_udp_motion_core

/tmp/b15_test_udp_motion_core
python3 -m compileall scripts
```

有 ROS2 和 SDK2 依赖时执行包构建与 CTest：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rk_go2_sdk_bridge \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rk_go2_sdk_bridge
colcon test-result --verbose
```

## 5. 同步与真机构建

从虚拟机仓库根目录同步 B1.5 文件：

```bash
rsync -avR \
  src/rk_go2_sdk_bridge/CMakeLists.txt \
  src/rk_go2_sdk_bridge/package.xml \
  src/rk_go2_sdk_bridge/include/rk_go2_sdk_bridge/udp_motion_core.hpp \
  src/rk_go2_sdk_bridge/src/udp_motion_core.cpp \
  src/rk_go2_sdk_bridge/src/go2_sdk_udp_server.cpp \
  src/rk_go2_sdk_bridge/test/test_udp_motion_core.cpp \
  src/rk_go2_sdk_bridge/scripts/cmd_vel_udp_forwarder.py \
  src/rk_go2_sdk_bridge/launch/go2_sdk_udp_bridge.launch.py \
  docs/B1_5_GO2_DETERMINISTIC_UDP_MOTION_BRIDGE.md \
  unitree@192.168.31.73:~/big-dog-league/
```

真机只构建目标包：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
colcon build --packages-select rk_go2_sdk_bridge \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

colcon test --packages-select rk_go2_sdk_bridge
colcon test-result --verbose

ros2 pkg prefix rk_go2_sdk_bridge
test -x \
  "$(ros2 pkg prefix rk_go2_sdk_bridge)/lib/rk_go2_sdk_bridge/go2_sdk_udp_server" \
  && echo "repository SDK server OK"
```

## 6. 真机人工测试

以下测试必须在至少 3m 平坦空旷区域进行。一人专门持遥控器，出现未按
预期停车、偏航或失去接管时立即急停。关闭巡线、迷宫、其他速度发布器
和旧 UDP 服务端。

终端 A：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch rk_go2_sdk_bridge go2_sdk_udp_bridge.launch.py \
  start_sdk_server:=true \
  sdk_network_interface:=eth0 \
  timeout_sec:=0.30 \
  max_vx:=0.25 \
  max_vy:=0.05 \
  max_yaw:=0.60
```

另一个终端使用以下命令检查服务端路径：

```bash
pgrep -af go2_sdk_udp_server
```

输出中的服务端路径必须位于：

```text
~/big-dog-league/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/
```

日志不得出现 `BalanceStand`，应出现：

```text
StopMove reason=startup ret=0
UDP server listening on 127.0.0.1:15001
```

终端 B 先验证零速：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10

ros2 topic pub -r 5 -t 5 --keep-alive 1.0 \
  /navigation/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, \
angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

零速日志必须直接出现 `StopMove reason=zero_command`，不得出现
`Move vx=0.04`。

再执行一次一秒前进：

```bash
ros2 topic pub -r 20 -t 20 --keep-alive 0.1 \
  /navigation/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25, y: 0.0, z: 0.0}, \
angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

服务端首个 `Move` 日志必须直接为 `vx=0.250000`。停止发布后应在
0.30 至 0.35 秒内出现 `StopMove`，且不得出现任何非零减速阶梯。

同一条件至少重复 10 次，逐次记录：

| 次数 | 是否迈步 | 总位移cm | 停车额外距离cm | 保持站立 | 3秒内重启 | 遥控接管 | PASS/FAIL |
|---:|---|---:|---:|---|---|---|---|
| 1 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 2 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 3 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 4 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 5 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 6 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 7 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 8 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 9 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |
| 10 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 | 未测 |

## 7. 当前结论

- 虚拟机纯逻辑测试：PASS。
- 虚拟机完整包构建：PASS。
- 新服务端真机编译：未确认。
- 新服务端真机起步重复性：未测试。
- 新服务端真实 watchdog 时间和停车距离：未测试。
- 遥控急停时间、距离及优先级：仍需人工测试。

在十次起步和停车测试全部通过前，不得将该桥接入迷宫自主运动输出。
