# ChatGPT Prompts for Maze Navigation Debugging

---

## Prompt 1: 系统架构诊断（优先问这个）

```
我在用 Unitree Go2 四足机器人跑迷宫全自主任务。机器人硬件配置：
- Go2 自带 LiDAR（原厂固定俯角朝下，frame_id=base_link）
- D435i 深度相机装在机械臂上（迷宫场景不可用）
- 主控是 Ubuntu 20.04 + ROS2 Foxy + CycloneDDS
- 运动控制链：ROS2 /navigation/cmd_vel → cmd_vel_udp_forwarder → UDP → go2_sdk_udp_server → Unitree SDK SportClient.Move()

遇到的问题是：运行 maze 脚本后机器人原地不动，脚本卡在 rclpy.spin_once 等不到传感器数据。具体现象：
1. 早期测试中 /utlidar/cloud_base 能收到 384 点/帧（x:-0.77~2.83, y:-0.93~0.82, z:-0.39~0.61）
2. 杀死部分进程并重启桥接层后，该 topic 不再有数据
3. 全代码库搜索找不到发布 /utlidar/cloud_base 的节点
4. 有一个 toggle_lidar.py（通过 Unitree SDK DDS 发 ON/OFF 指令到 rt/utlidar/switch）

请帮我分析：
a) /utlidar/cloud_base 的数据源最可能是什么？（ROS2 驱动节点？DDS→ROS2 bridge？Go2 固件直发？）
b) 如何确定当前该 topic 是否有发布者？如果发布者不存在，可能的启动方式是什么？
c) Go2 的 LiDAR 数据流架构通常是什么样的？（Unitree SDK DDS → ??? → ROS2）
d) 如何系统性地排查 "topic 曾经有数据、现在消失" 这类问题？
```

---

## Prompt 2: 拐角检测逻辑问题

```
我的迷宫机器人用 LiDAR 侧向扇区（30°-60°）的中值 y 距离作为侧向 clearance。
拐角检测条件是：前方距离 < 0.40m 且 侧向 clearance > 0.50m。

机器人参数：
- Go2 车身宽约 0.36m
- LiDAR 装在顶端正前方，俯角朝下
- 迷宫墙高 45cm

实测发现侧向 clearance 始终为 0.00（z 过滤已修复后曾出现 0.23-0.24m）。

请分析：
a) 在标准迷宫走廊（宽 0.6-0.8m）中，侧向 clearance 的合理范围是多少？
b) 拐角检测阈值 0.50m 是否物理上可达到？
c) 更鲁棒的拐角检测策略应该怎么做？（比如用 clearance 的变化率、扇形点密度差等）
d) 在侧向检测不可靠的情况下，有哪些替代的拐角/路口检测方法？
```

---

## Prompt 3: 地面反射与 LiDAR 俯角补偿

```
Unitree Go2 的 LiDAR 原厂俯角朝下，安装在机器人顶部约 35cm 高处。
点云数据（frame_id=base_link，迷宫入口处）：
  x: -0.777 ~ 2.826
  y: -0.931 ~ 0.815
  z: -0.389 ~ 0.605
  其中 z>0.005 仅 17/384 个点

迷宫墙高 45cm。

请分析：
a) LiDAR 俯角安装对点云 z 分布的影响
b) 如何区分地面反射点和墙壁点（仅凭 z 值 + x-y 位置）
c) 已知 LiDAR 俯角和安装高度的情况下，能否对点云做坐标变换补偿？
d) 如果不能物理调整 LiDAR 角度，软件层面有什么方法最大化墙壁检测能力？
```

---

## Prompt 4: CycloneDDS 多域配置

```
ROS2 Foxy + CycloneDDS 环境中，我需要一个 ROS2 节点同时：
- 订阅域 A（默认域 0）的 LiDAR/odom/imu 话题
- 发布到域 B（域 10）的 /navigation/cmd_vel 话题
（域 10 是 cmd_vel_udp_forwarder 所在的域）

CycloneDDS 版本较旧（< 0.10），不支持 <SharedMemory> XML 配置。
启动时出现 std::bad_alloc 错误。

请问：
a) 单个 rclpy 节点能否跨 DDS 域通信？如果不能，推荐什么方案？
b) ros2 domain_bridge 是否适合这个场景？如何配置？
c) 有没有办法在不改 bridge launch 文件的情况下统一所有节点到一个域？
d) std::bad_alloc 在 CycloneDDS 中的常见原因和解决方案？
```

---

## Prompt 5: maze_full_auto.py 代码审查

```
以下是一个迷宫全自主导航脚本（Python + ROS2），用于 Unitree Go2 机器人。
请审查其中的逻辑问题、边界条件和潜在的运行时风险。

重点关注：
1. CRUISE 阶段的速度分级和 corner 检测逻辑
2. TURN 阶段的转弯完成判断（纯 IMU/odo 积分，无传感器反馈）
3. 多线程设计（publish_loop daemon + 主循环）的竞态条件
4. 各种 EMERG/BACKUP 状态之间的转换是否正确
5. side_blocked 判断是否会导致死锁

[以下粘贴 maze_full_auto.py 完整代码]
```
