# 迷宫全自主技术报告 — 2026-08-07

## 一、当前进度总览

| 阶段 | 状态 | 说明 |
|------|------|------|
| 环境隔离 | ✅ 已解决 | 确认需要停比赛栈(command_mux)，否则与 maze 脚本抢 `/navigation/cmd_vel` |
| 速度上限 | ✅ 已修复 | `VX_CRUISE 0.30→0.25`，不超过 forwarder 的 max_vx=0.25 |
| z 轴过滤 | ✅ 已修复 | `p.z > 0.005` → `p.z > -0.40`，LidarDistanceConfig obstacle_z_min 同步下调 |
| raw_front 误触发 | ✅ 已移除 | 地面反射导致 median 极小→EMERG 死锁，已删除 raw check |
| DDS 通信 | 🔴 未解决 | maze 脚本与 LiDAR/forwarder 间的 DDS 发现失败 |
| 桥接层启动 | 🔴 未解决 | forwarder 因 CycloneDDS XML 不兼容崩溃 |
| 直走 CRUISE | ❌ 未验证 | 机器人从未真正移动过 |
| 拐角检测 | ❌ 未测试 | L/R 侧向在 z 修复后曾输出 0.23/0.24，但因 EMERG 锁死未进入前进状态 |
| 转弯 TURN | ❌ 未测试 | 从未进入过 |
| 五弯跑完 | ❌ 未测试 | — |

**结论：迷宫测试处于 Phase 0 — 传感器数据链路尚未打通。**

---

## 二、已完成的修复

### 2.1 VX_CRUISE 超限 (scripts/maze_full_auto.py:18)
- **问题**：`VX_CRUISE = 0.30`，cmd_vel_udp_forwarder 的 max_vx=0.25，发 0.30 被判定为 out_of_range 立即 stop
- **修复**：改为 0.25

### 2.2 z 轴过滤丢失 95.6% 点云 (scripts/maze_full_auto.py:30-36, 86, 98)
- **问题**：Go2 自带 LiDAR 俯角朝下，墙点 z 集中在 [-0.35, 0.1]；`p.z > 0.005` 一刀切只保留 17/384 个点
- **实测数据**（迷宫入口，frame_id=base_link）：
  ```
  点数: 384
  x: -0.777 ~ 2.826
  y: -0.931 ~ 0.815
  z: -0.389 ~ 0.605
  z>0.005: 17  |  z<=0.005: 367
  ```
- **修复**：
  - 导航路径 `p.z > 0.005` → `p.z > -0.40`
  - LidarDistanceConfig: `obstacle_z_min_m: 0.03 → -0.40`, `ground_z_min_m: -0.50`, `ground_z_max_m: -0.42`

### 2.3 raw_front 误触发 EMERG (已移除)
- **问题**：放开 z 过滤后地面反射点混入 raw_front 计算，median 被拉到 0.05-0.24m，每帧触发 EMERG stop + sleep(0.3)，机器人完全锁死
- **修复**：删除 raw-front-every-frame 的 EMERG 检测（CRUISE 段已有基于过滤后 `front` 的 EMERG 保护，第 129 行）

### 2.4 competition stack topic 冲突 (已确认方案)
- **问题**：command_mux_node 以 20Hz 往 `/navigation/cmd_vel` 发零速度，与 maze 脚本交替到达 forwarder
- **方案**：运行 maze 前必须停掉比赛栈

---

## 三、未解决的阻塞问题

### 3.1 🔴 CycloneDDS 配置兼容性
**文件**: `config/maze_cyclonedds.xml`
**现象**: forwarder 启动时报 `SharedMemory: unknown element`，然后 `rmw_create_node: failed to create domain`，进程崩溃
**原因**: Go2 上的 CycloneDDS 版本 < 0.10，不支持 `<SharedMemory>` 元素（已删除该段），但仍不确定当前 XML 是否完全兼容
**当前状态**: CycloneDDS XML 已更新（移除 SharedMemory），但尚未在机器人上验证

### 3.2 🔴 ROS_DOMAIN_ID 不一致
**文件**: `scripts/run_maze.sh:4`, `go2_sdk_udp_bridge.launch.py:15`
**现象**: 
- 桥接层 forwarder 硬编码 `ROS_DOMAIN_ID=10`
- LiDAR 数据 (`/utlidar/cloud_base`) 在 domain 0（默认域）可用
- maze 脚本需要在同一域同时收 LiDAR 和发 cmd_vel
**冲突**: 如果 maze 用 domain 0 → forwarder 收不到 cmd_vel；如果 maze 用 domain 10 → 收不到 LiDAR
**当前状态**: run_maze.sh 设为 domain 10，run_maze.sh 设为 domain 10（但 LiDAR 数据源域未知）

### 3.3 🔴 LiDAR 数据源丢失
**现象**: 
- 早期运行中 `/utlidar/cloud_base` 正常发布（384 点，frame_id=base_link）
- 当前运行中该 topic 无数据，maze 脚本在 `spin_once` 中无限等待
**未知**:
- 哪个进程/节点发布 `/utlidar/cloud_base`？（全代码库搜索无果）
- LiDAR 是否需要显式 toggle ON？（`toggle_lidar.py ON --network eth0`）
- LiDAR 数据是否为 Go2 固件直发（通过 Unitree DDS → ROS2 bridge）？
- 如果是固件直发，ROS2 bridge 是哪个进程？

### 3.4 🔴  maze 脚本无 CRUISE 输出
**现象**: 最近一次运行只打印了启动横幅和初始状态，无任何 CRUISE 行
**轨迹**: 卡在 `rclpy.spin_once(chk, timeout_sec=0.01)` — 订阅回调从未触发 → 无可用的点云/odom/imu 数据
**根因**: LiDAR 数据源不可用（见 3.3），且可能存在 DDS 发现层面的问题（`std::bad_alloc` 错误）

---

## 四、maze_full_auto.py 代码层面的问题

### 4.1 🔴 侧向拐角检测阈值的可达性
**代码**: lines 121-127
```python
left_cl = sorted(left_pts)[len(left_pts)//2] if left_pts else 0.0
right_cl = sorted(right_pts)[len(right_pts)//2] if right_pts else 0.0
turn_left_ready = (0.01 < front < 0.40) and (left_cl > 0.50)
turn_right_ready = (0.01 < front < 0.40) and (right_cl > 0.50)
```
**问题**: Go2 车身宽 ~0.36m，迷宫走廊宽 ~0.6-0.8m 时，理论侧向 clearance = (0.6-0.36)/2 = 0.12m。`left_cl > 0.50` 要求走廊宽 > 1.36m，在标准迷宫中几乎不可能。**即使 LiDAR 正常工作，拐角检测也永远不会触发。**
**建议**: 将 0.50 降至 0.25-0.30，或改用侧向 clearance 的相对变化率（Δleft_cl）检测拐角

### 4.2 🔴 BACKUP 死循环
**代码**: lines 135-142
```python
side_blocked = (left_cl < 0.25 and right_cl < 0.25)
if 0.01 < front < STOP_DIST and side_blocked:
    tw.linear.x = -0.10  # BACKUP
```
**问题**: 迷宫走廊中 L/R 常 < 0.25m → side_blocked 恒 True → 一旦 front < 0.35m 就 BACKUP → 后退至 front > 0.35m 又前进 → 前后振荡
**建议**: 在 side_blocked 且 front 近时，加入原地旋转逻辑尝试寻找开口

### 4.3 🟡 朝向控制器增益过低
**代码**: line 159
```python
tw.angular.z = min(0.10, max(-0.10, heading_deg * 0.005))
```
**问题**: 8° 偏差只产生 0.04 rad/s 修正，实测中朝向漂移 -8.1° → +6.3° 无法纠正
**建议**: 增大 P 增益至 0.02-0.05，或启用项目中已有的 `HeadingController` 模块

### 4.4 🟡 TURN 状态无超时/无 LiDAR 反馈
**代码**: lines 164-183
- 转弯 90° 纯靠 IMU 积分 + odom，无 LiDAR 墙反馈
- 如果 IMU/odom 卡死，`remaining < 15.0` 永远不会满足 → 无限旋转
- 无最小/最大转弯时间限制

### 4.5 🟡 启动瞬间 publish_loop 先发零速度
**代码**: lines 68-80
`current_twist` 初始化为全零 Twist，daemon 线程立即以 50Hz 发布。主循环需等到第一个偶数帧才设置非零 `current_twist`。forwarder 收到零→StopMove→然后收到非零→Move，可能导致机器人起步抖动。

### 4.6 🟡 frame skip 减半控制率
**代码**: line 91
`if frame[0] % 2 != 0: continue` — 有效控制率从 ~100Hz 降至 ~50Hz

### 4.7 🟡 120 秒硬超时
**代码**: line 83
5 弯 * (直走 + 转弯) 可能需要 > 120s

### 4.8 🔵 现有 HeadingController/LidarWallExtractor 未被充分利用
- `hc` (HeadingController) 初始化后从未调用其控制输出
- `we` (LidarWallExtractor) 的 `build_corridor_model` 只取了 `corridor_heading`

---

## 五、系统架构层面的问题

### 5.1 传感器硬件限制
- **LiDAR**: Go2 自带，俯角朝下（原厂固定），FOV 有限，主用于地面障碍物检测而非墙壁感知
- **D435i 深度相机**: 装在机械臂上，迷宫场景不可用（机械臂未部署）
- **无侧面/后方传感器**: 迷宫墙壁检测仅依赖前向 LiDAR 的侧向扇区

### 5.2 DDS 中间件碎片化
- 三套 DDS 共存：CycloneDDS (ROS2)、Unitree SDK DDS (机器人控制/传感器)、FastRTPS (可能残留)
- 不同节点使用不同 DDS domain，缺乏统一配置
- CycloneDDS 版本较旧 (< 0.10)，不支持部分 XML 配置特性

### 5.3 启动依赖链不清晰
```
toggle_lidar.py ON → [未知bridge] → /utlidar/cloud_base
                                      /utlidar/robot_odom
                                      /utlidar/imu
                                         ↓
go2_sdk_udp_bridge.launch.py → forwarder 订阅 /navigation/cmd_vel
                             → sdk_server 监听 UDP 127.0.0.1:15001
                                         ↓
maze_full_auto.py 订阅 LiDAR topics → 发布 /navigation/cmd_vel → forwarder → UDP → SDK → 机器人
```
当前：`[未知bridge]` 缺失，整个链路断开。

---

## 六、潜在风险（尚未遇到但可能发生）

| # | 风险 | 触发条件 |
|---|------|----------|
| 1 | 迷宫墙高 45cm，但 LiDAR 俯角导致远处墙顶低于传感器平面 → 远墙不可见 | 走廊 > 2m 时 |
| 2 | Go2 步态在慢速 (0.08-0.15 m/s) 下可能不稳定 | speed gradient 的 0.08 档 |
| 3 | TURN 时 vx=0.10 + wz=0.50 同时存在，可能导致侧滑/摔倒 | 转弯执行 |
| 4 | LiDAR 盲区：< 0.08m (min_range) 无法检测贴脸障碍 | 靠墙太近时 |
| 5 | IMU 漂移积分累积 → 90° 转弯实际可能只有 70-80° | 长时间测试 |

---

## 七、已修改的文件清单

| 文件 | 修改内容 |
|------|----------|
| `scripts/maze_full_auto.py:18` | `VX_CRUISE = 0.30` → `0.25` |
| `scripts/maze_full_auto.py:30-36` | `LidarDistanceConfig` 自定义 z 阈值 |
| `scripts/maze_full_auto.py:85-91` | 删除 raw-front-every-frame EMERG 检测 |
| `scripts/maze_full_auto.py:98` | `p.z > 0.005` → `p.z > -0.40` |
| `config/maze_cyclonedds.xml` | 新建 CycloneDDS 配置文件 |
| `scripts/run_maze.sh` | 新建统一启动脚本 |
