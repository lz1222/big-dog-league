# Codex 交接提示词 — 迷宫项目 2026-08-07

---

## Prompt 1: 环境诊断（优先）

```
我在 Unitree Go2 EDU 四足机器人上跑 ROS2 Foxy 迷宫导航。
环境变量设置如下：

export LD_LIBRARY_PATH=/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/thirdparty/lib/aarch64:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

关键约束：必须用 Unitree SDK 自带的 CycloneDDS 库（路径如上），否则 ROS2 在
Go2 内部网卡 eth1 上段错误。

问题：在相同环境变量下——
- tools/maze_sensor_probe.py（简单 Node + 3 订阅 + MultiThreadedExecutor）
  在任何终端都能正常收到传感器数据
- scripts/maze_full_auto_v2.py（较复杂 Node，额外 import 了 rk_maze 包：
  lidar_distance_core, lidar_wall_extractor, local_occupancy_grid,
  heading_controller）
  在 Bash 工具进程中能正常工作并输出传感器数据，但在用户交互终端中传感器
  callback 完全不触发（CLOUD_AGE=infs ODOM_AGE=infs IMU_AGE=infs）

已验证：libddsc 加载正确、rk_maze 导入正常、rclpy.init() 无报错、
QoS 配置相同（BEST_EFFORT, VOLATILE, depth=10）、MultiThreadedExecutor 
配置相同（3 threads）。

请分析可能的原因：
1. rk_maze 包的导入是否可能影响 DDS 发现或 rclpy 初始化？
2. 交互终端 vs 非交互终端的 Python 行为差异？
3. 是否需要调整 import 顺序？
4. 是否需要检查 PYTHONPATH 或 AMENT_PREFIX_PATH 的差异？
```

---

## Prompt 2: ROS2 Timer vs Python Thread 控制循环

```
我在 maze_full_auto_v2.py 中遇到 ROS2 Timer 问题。

原来的设计：
self._plan_timer = self.create_timer(1.0 / 20.0, self._on_plan_timer)

现象：plan_timer 只触发 1-2 次然后停止，cmd_timer（50Hz）正常。
两个 Timer 在同一次 __init__ 中创建，配置相同。

临时方案：用 Python threading.Timer 替代 ROS2 Timer：
self._plan_thread = threading.Thread(target=self._plan_loop, daemon=True)
def _plan_loop(self):
    while self._plan_running:
        self._on_plan_timer_impl()
        time.sleep(0.05)

MultiThreadedExecutor(num_threads=3) + executor.spin(daemon thread)

问题是：为什么 ROS2 Timer 在 MultiThreadedExecutor 中会停止触发，
而 cmd_timer 正常？Node 有 3 个订阅（PointCloud2/Imu/Odom at 
BEST_EFFORT QoS）+ 1 个发布者 + 2 个 Timer。是在高频回调
（Odom 150Hz + IMU 200Hz）下被饿死了吗？
```

---

## Prompt 3: 迷宫状态机逻辑审查

```
以下是一个迷宫导航状态机的 Dry Run 输出。机器人在迷宫入口，前方有墙，
左侧有开口。请分析状态转换是否合理：

[1] SYSTEM_PRECHECK → SENSOR_STALE
SENSOR_STALE CLOUD_AGE=infs ODOM_AGE=infs IMU_AGE=infs
[2] SENSOR_STALE → WAIT_FOR_SENSORS
[3] WAIT_FOR_SENSORS → CRUISE
[4] CRUISE → EMERGENCY_STOP (front=0.14m)
[6] EMERGENCY_STOP → RECOVERY_DECISION
[7] RECOVERY_DECISION → TURN_APPROACH (L=0.83m, right blocked)
[15] TURN_APPROACH → SENSOR_STALE (cloud 断流 3s)
[17] SENSOR_STALE → WAIT_FOR_SENSORS → CRUISE
[19] CRUISE → EMERGENCY_STOP (front=0.18m)

路线: LEFT-LEFT-RIGHT-RIGHT-LEFT, 每个弯90度。

需要审查：
1. 入口处 front=0.14m 即触发 EMERGENCY，但这可能只是离墙太近而非真正的紧急情况。
   是否应该区分 "wall proximity" 和 "imminent collision"？
2. TURN_APPROACH 后因 cloud 断流回到 SENSOR_STALE，丢失了转弯状态。
   恢复后如何重新进入转弯？
3. cloud 偶尔断流 3-5s（LiDAR 自身特性），cloud_stop_age=2s 是否太激进？
4. RECOVERY_DECISION 中检测左侧开口就转 TURN_APPROACH 是否正确？
   第一个弯是 LEFT，但开口可能在任意一侧。
5. 拐角检测条件：连续 5 帧 opening > 0.30m 确认。入口 L=0.83m 远超阈值，
   但 front=0.14m 触发了 EMERGENCY 而不是 CORNER_CANDIDATE。这个优先级对吗？
```

---

## Prompt 4: CycloneDDS 版本冲突解决方案

```
Unitree Go2 机器人的机载 Jetson 上有三个 CycloneDDS 版本：

1. ROS2 Foxy 自带: /opt/ros/foxy/lib/aarch64-linux-gnu/libddsc.so.0.7.0
2. 系统安装: /usr/local/cyclonedds/lib/libddsc.so.0.10.2
3. Unitree SDK 自带: third_party/unitree_sdk2_official/thirdparty/lib/aarch64/libddsc.so

问题：
- ROS2 rclpy 用 #1 或 #2 在 Go2 内部网卡 eth1 上会段错误
- 只有 #3 能正常工作
- 必须设置 LD_LIBRARY_PATH 优先指向 #3
- 但 ros2 launch 启动的 forwarder 进程继承了不同的 LD_LIBRARY_PATH

当前方案：所有 ROS2 命令前手动 export LD_LIBRARY_PATH。

请设计一个更优雅的解决方案：
1. 能否通过 ldconfig 或 /etc/ld.so.conf.d/ 设置系统级优先级？
2. 能否修改 ros2 launch 的 forwarder 环境？
3. 能否创建 wrapper 脚本自动注入正确的 LD_LIBRARY_PATH？
4. 有没有办法让 rclpy 静态链接正确的 CycloneDDS？
```

---

## Prompt 5: 最小复现脚本

```
请帮我写一个最小诊断脚本，用于排查 "maze v2 在交互终端传感器回调不触发" 的问题。

脚本要求：
1. 逐步增加复杂度，每步输出 PASS/FAIL
2. Step 1: 纯 rclpy 订阅 cloud_base + MultiThreadedExecutor
3. Step 2: 加入 rk_maze imports
4. Step 3: 加入 LidarWallExtractor 初始化
5. Step 4: 加入 LocalOccupancyGrid 初始化
6. Step 5: 加入 Python 后台规划线程
7. Step 6: 完整 maze v2 Node 初始化

每步等待 3 秒检查 cloud callback 是否触发。输出哪一步开始 FAIL。

环境变量：
export LD_LIBRARY_PATH=/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/thirdparty/lib/aarch64:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
source /opt/ros/foxy/setup.bash
```
