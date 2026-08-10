# 迷宫项目进度汇报 & Codex 交接提示词 — 2026-08-07

---

## 一、项目目标

Unitree Go2 EDU 四足机器人全自主迷宫导航：直走→拐角检测→转弯→五弯→出口。
路线：LEFT-LEFT-RIGHT-RIGHT-LEFT（每个弯 90°）。

---

## 二、硬件环境

| 项目 | 详情 |
|------|------|
| 机器人 | Unitree Go2 EDU |
| 机载电脑 | Jetson Orin NX, Ubuntu 20.04, ROS2 Foxy |
| LiDAR | Go2 原厂固件直发，frame_id=base_link, ~10Hz, ~600pts/frame |
| 传感器 | `/utlidar/cloud_base` + `/utlidar/robot_odom` + `/utlidar/imu` |
| 深度相机 | Intel D435i 装在机械臂上（迷宫不可用） |
| DDS | CycloneDDS（必须用 Unitree SDK 自带的版本） |
| 内部网络 | eth1 = 192.168.123.18/24，直连 Go2 嵌入式控制器 |
| 运动链 | ROS2 /navigation/cmd_vel → forwarder → UDP → SDK server → SportClient.Move() |

---

## 三、关键环境配置

```bash
# 必须按此顺序
export LD_LIBRARY_PATH=/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/thirdparty/lib/aarch64:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI
```

**核心发现**：ROS2 Foxy 原装 CycloneDDS v0.7.0 和系统 v0.10.2 在 Go2 内部网卡上都会段错误。必须优先加载 Unitree SDK 自带的 CycloneDDS（`third_party/unitree_sdk2_official/thirdparty/lib/aarch64/libddsc.so`）。

---

## 四、已完成工作

### 4.1 网络和数据链
- eth1 永久配置为 192.168.123.18/24（NetworkManager `go2-internal`）
- Unitree SDK DDS 和 ROS2 DDS 均在 eth1 上验证通过
- 三个传感器 topic 全部通过 30s 连续探针测试

### 4.2 运动链
- SDK server 在 eth1 上初始化成功（需先 Damp→StandUp 进入 Sport 模式）
- forwarder 的 CycloneDDS 段错误已修复（launch 文件 LD_LIBRARY_PATH）
- forwarder 的 ROS_DOMAIN_ID 硬编码已移除（从环境继承）
- 端到端命令验证：`ros2 topic pub vx=0.1 → Move ret=0`

### 4.3 代码产出

| 文件 | 说明 |
|------|------|
| `scripts/maze_full_auto_v2.py` | 重构迷宫控制器（见第五节） |
| `scripts/maze_full_auto.py` | 原始版本（spin_once 架构，Deprecated） |
| `tools/maze_sensor_probe.py` | 60s 传感器探针，只读 |
| `tools/lidar_dds_bridge.py` | DDS→ROS2 LiDAR 桥接（备用，实际不需要） |
| `scripts/maze_startup.sh` | 一键启动脚本 |
| `scripts/run_maze.sh` | maze v2 独立启动脚本 |
| `config/maze_cyclonedds.xml` | CycloneDDS 配置（auto 接口） |

---

## 五、maze_full_auto_v2.py 架构

### 5.1 架构变更（vs 原始版本）

| 特性 | v1 (旧) | v2 (新) |
|------|---------|---------|
| ROS2 架构 | `while + rclpy.spin_once()` | Node + MultiThreadedExecutor |
| 控制循环 | spin_once 阻塞等待 | Python 后台线程 20Hz |
| 命令发布 | daemon thread + while True | ROS2 Timer 50Hz + 过期看门狗 |
| 传感器数据 | 共享 list (非线程安全) | SensorSnapshot 不可变快照 + lock |
| STALE 保护 | 无 | cloud/odom/imu 独立 warn/stop age |
| 命令过期 | 无（无限重放旧命令） | cmd_max_age=0.2s 超时→ZERO |
| 侧向检测 | median(p.y), 无点=0 | SideObservation(valid=False, reason) |
| OccupancyGrid | 永久累积历史点 | 每帧清空（MODE A） |
| 拐角检测 | 固定阈值 0.50m | OpeningEvidence 累积确认 |
| 转弯 | 纯 IMU 积分（会重复积分旧样本） | Odom signed yaw + 方向符号校验 |
| 路线推进 | 转完就 route++ | 需 CORRIDOR_REACQUIRE 确认新走廊 |
| EMERGENCY | sleep(0.3) + continue 死循环 | 独立状态 → RECOVERY_DECISION |
| BACKUP | 无限 vx=-0.10 | BACKUP → STOP_AND_REASSESS（无后方感知时禁用） |
| Dry Run | 无 | MazeConfig.dry_run=True 强制零输出 |

### 5.2 状态机（14 状态）

```
SYSTEM_PRECHECK → WAIT_FOR_SENSORS → CRUISE → CORNER_CANDIDATE
→ TURN_APPROACH → ARC_TURN_MAIN → TURN_FINE_ALIGN
→ CORRIDOR_REACQUIRE → CRUISE（下一弯）
EMERGENCY_STOP → RECOVERY_DECISION → BACKUP → STOP_AND_REASSESS
SENSOR_STALE → WAIT_FOR_SENSORS（恢复后）
FAULT_STOP / DONE
```

### 5.3 Dry Run 实测数据（迷宫入口）

```
front=0.14~0.20m  （前墙很近）
L=0.71~1.22m      （左侧开口）
R=0.37~0.56m      （右侧有墙）
hdg=-21°~+0°      （朝向偏转）
状态流: CRUISE → EMERGENCY_STOP → RECOVERY → TURN_APPROACH
```

---

## 六、当前阻塞问题

### P0: maze v2 在用户终端传感器 AGE=infs

**现象**：从 Bash 工具进程启动 maze v2 正常接收传感器数据，从用户终端启动则所有传感器 AGE=infs。

**已验证**：
- 传感器探针在两种环境下都正常
- 最小测试节点在两种环境下都正常
- maze v2 在 Bash 工具进程正常运行（front=0.14m L=1.22m 等）
- 环境变量（LD_LIBRARY_PATH, RMW_IMPLEMENTATION, ROS_DOMAIN_ID）配置正确

**怀疑方向**：
1. `ros2 launch` 命令修改了子进程环境（PYTHONPATH/AMENT_PREFIX_PATH），污染了后续 maze v2 的运行
2. `source install/setup.bash` 与 ROS2 Foxy 的 sensor_msgs 版本冲突
3. Python GIL/线程调度在用户终端和 Bash 工具进程中行为不同

### P1: 未进行真机运动测试
- maze v2 的 armed 模式未测试
- 实际转弯未在迷宫中验证
- 五弯全流程未跑通

---

## 七、未完成的代码工作

1. Cloud 偶尔断流 3-5s，cloud_stop_age=2s 太短，需调到 5s
2. BACKUP 无后方感知（需加 rear LiDAR coverage 检查）
3. TURN 状态无超时保护（可能无限旋转）
4. maze v2 的 ROS2 Timer 在用户终端不触发（临时用 Python 线程替代）
5. 传感器 QoS 未做发布者兼容性确认（BEST_EFFORT vs RELIABLE）
6. OccupancyGrid 未实现 Odom 补偿模式（MODE B）
7. 地面 z 过滤仍用固定阈值（-0.40），未做自适应直方图估计
8. `maze_startup.sh` 中的 StandUp 只在 mode=0 时执行，但 mode 检查不可靠
