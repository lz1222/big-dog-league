# B0 Hardware Capability Audit

## 1. 审计信息

- 审计日期：2026-07-26
- 审计范围：`src/`、`scripts/`、`docs/`、`third_party/` 中的源码、接口定义、配置和说明文档
- 排除项：`build/`、`install/`、`log/` 等生成物不作为接口实现证据
- 审计方式：静态仓库扫描；本次未连接 Go2、机械臂、雷达或 D435i，未执行真机动作
- 约束：本次只生成审计文档，未修改比赛功能代码

状态定义：

| 状态 | 含义 |
|---|---|
| 已实现 | 仓库内存在可执行的软件链路，但仍可能需要真机验收 |
| 部分实现 | 入口和部分链路存在，但缺少真实硬件调用、反馈闭环或生产者 |
| 仅声明 | 上层会调用或配置中出现，但执行端没有实现 |
| 外部依赖 | 本仓库只保留调用端，关键实现位于仓库外 |
| 缺失 | 未找到对应接口或处理逻辑 |

## 2. 审计结论

**B0 静态审计已完成，但当前硬件能力不能判定为“真机就绪”。**

主要结论：

1. Go2 已有 ROS 2 Sport Request、直接 Unitree SDK2 和 UDP 转发三类控制路径，但没有统一的硬件反馈闭环。
2. `start_jump`、`finish_jump` 和 `avoid_zone` 有 ROS 2 Action 入口；前两个当前只是相同的两段低速前进序列，不是 `SportClient.FrontJump()`。
3. `stairs_up_down` 只在任务状态机和文档中出现，运动 Action 服务器没有映射或执行逻辑。
4. 机械臂有 adapter、SDK bridge、command topic 和 status topic，但真实厂家 SDK 调用仍是 TODO；D1 DDS 示例没有接入任务节点。
5. `LaserScan` 有消费者但没有仓库内生产者，且默认关闭。
6. Unitree vendored ROS 2 栈说明了 `/utlidar/cloud` 点云能力，但项目代码没有 `PointCloud2` 订阅、转换或避障接入。
7. D435i depth image 的启动、订阅、解析和距离提取已经存在，是传感器侧最完整的软件接口。

## 3. 当前已有硬件接口（总体能力矩阵）

| 域 | 接口/能力 | 当前状态 | 关键证据 | 主要缺口 |
|---|---|---|---|---|
| Go2 运动 | `/locomotion/execute_motion` (`ExecuteMotion`) | 部分实现 | `rk_interfaces/action/ExecuteMotion.action`；`gait_control_node.py` | 部分 motion name 未实现；无真实动作完成反馈 |
| Go2 适配器 | `RobotMotionAdapter` -> `/navigation/cmd_vel` | 已实现 | `rk_locomotion/rk_locomotion/gait_control_node.py:56` | `vy`、body height、recovery stand 未接到底层 |
| Go2 ROS 2 驱动 | `/navigation/cmd_vel` -> `/api/sport/request` | 部分实现 | `cmd_vel_bridge_node.py`；`go2_motion_client.py` | 默认 backend 是 mock；仅支持 `vx`、`vyaw`；无状态订阅 |
| Go2 UDP 驱动 | `/navigation/cmd_vel` -> UDP `15001` | 外部依赖 | `cmd_vel_udp_forwarder.py` | `go2_sdk_udp_server` 源码不在仓库内 |
| Go2 直接 SDK2 | `SportClient`、`VuiClient`、`VideoClient` | 已实现 | `rk_go2_sdk_bridge/src/*.cpp` | 主要是命令行工具；未统一接入 `ExecuteMotion` |
| `start_jump` | ROS 2 motion name | 部分实现 | `gait_control_node.py:732,1274` | 只是开环前进序列；未调用 `FrontJump()` |
| `finish_jump` | ROS 2 motion name | 部分实现 | `gait_control_node.py:734,1274` | 与 `start_jump` 使用相同参数和执行逻辑 |
| `stairs_up_down` | ROS 2 motion name | 仅声明 | `mission_state_machine_node.py:425` | Action 服务器无映射、无执行器、无恢复逻辑 |
| `avoid_zone` | ROS 2 motion name | 部分实现 | `gait_control_node.py:737,1088` | 主要为定时/距离触发路线；雷达默认未接通 |
| 新机械臂 adapter | `ArmHardwareAdapter`、`SdkBridgeArmAdapter` | 部分实现 | `rk_arm_control/adapters/` | 只有通用 JSON 输出，没有真实硬件确认 |
| 新机械臂 SDK bridge | command/status JSON bridge | 部分实现 | `new_arm_sdk_bridge_node.py` | real move/gripper/stop 全部是 TODO |
| D1 adapter | `UnitreeD1SdkAdapter` | 仅声明 | `d1_pick_node.py:246` | 不发布 `rt/arm_Command`；夹爪和 stop 未实现 |
| D1 DDS 示例 | `rt/arm_Command` 和反馈 topics | 已实现 | `third_party/unitree_d1_sdk/src/` | vendored 示例未接入 ROS 2 arm Action |
| Depth image | D435i `sensor_msgs/Image` | 已实现 | `realsense_low_bandwidth.launch.py`；`depth_wall_distance_node.py` | 未做本次真机帧率、量程和安装位姿验收 |
| LaserScan | `/scan` (`sensor_msgs/LaserScan`) 消费者 | 部分实现 | `gait_control_node.py:419` | 无驱动/发布者；默认 `enable_scan: false` |
| PointCloud | `/utlidar/cloud` 能力线索 | 外部依赖 | `third_party/unitree_ros2/README.md:342` | 项目代码没有 `PointCloud2` 接口或 cloud-to-scan |
| Go2 姿态/状态 | low state/high state/IMU | 缺失 | `gait_control_node.py:1902` 的 TODO | 稳定性检查当前固定返回 `True` |

## 4. Unitree SDK 调用位置

### 4.1 项目自有 Go2 调用

| 位置 | 调用方式 | 能力 |
|---|---|---|
| `src/rk_go2_sdk_bridge/src/go2_sdk_motion_action.cpp` | `ChannelFactory` + `SportClient` + `VuiClient` | `BalanceStand`、`ClassicWalk`、`StaticWalk`、`TrotRun`、`FreeWalk`、`StandUp`、`EconomicGait`、`FrontJump`、`Hello`、`Stretch`、`RecoveryStand`、`StopMove`、前灯亮度 |
| `src/rk_go2_sdk_bridge/src/go2_sdk_velocity_action.cpp` | `SportClient.Move()` / `StopMove()` | 定时直行、转向和停止；`vy` 固定为 0 |
| `src/rk_go2_sdk_bridge/src/go2_sdk_speed_sweep.cpp` | `SportClient.Move()` / `StopMove()` | 线速度或角速度扫速诊断 |
| `src/rk_go2_sdk_bridge/src/go2_sdk_capture_image.cpp` | `VideoClient.GetImageSample()` | 抓取 Go2 前置相机 JPEG |
| `src/rk_unitree_driver/rk_unitree_driver/go2_motion_client.py` | `unitree_api/msg/Request` | `/api/sport/request` 上发布 Move `1008` 和 StopMove `1003` |
| `src/rk_tools/rk_tools/obstacle_direct_route_node.py` | 启动 `go2_sdk_motion_action`，可选发布 Sport Request | 直接动作优先走 SDK helper；ROS topic fallback 默认关闭 |
| `src/rk_tools/rk_tools/keyboard_route_node.py` | 启动 SDK motion/velocity helper | 键盘记录和回放直接 SDK 动作 |
| `src/rk_mission/rk_mission/sign_action_executor_node.py` | 启动 `go2_sdk_motion_action` | 警示牌动作 `stretch`、`hello`、闪灯 |

`obstacle_direct_route_node.py` 还定义了 Sport API ID：

| 动作 | API ID |
|---|---:|
| `balance_stand` | 1002 |
| `stop_move` | 1003 |
| `stand_up` | 1004 |
| `recovery_stand` | 1006 |
| `front_jump` | 1031 |
| `economic_gait` | 1063 |

### 4.2 Go2 构建和外部依赖

- `src/rk_go2_sdk_bridge/CMakeLists.txt` 仅在找到 `unitree_sdk2` 时构建四个直接 SDK 工具。
- 审计时本地 `install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/` 中四个工具均存在且可执行；这只能证明已构建，不能证明真机可用。
- `cmd_vel_udp_forwarder.py` 依赖外部 `go2_sdk_udp_server`。默认路径指向 `/home/unitree/unitree_go2_sdk_test/build/go2_sdk_udp_server`，仓库内没有该 server 源码，因此该路径不可独立复现。
- ROS 2 Request 路径依赖 vendored `unitree_ros2` 的 `unitree_api/msg/Request`、CycloneDDS 配置、正确网卡和兼容 RMW。

### 4.3 D1 Unitree SDK2 调用

`third_party/unitree_d1_sdk/src/` 中存在直接 DDS 示例：

- `joint_angle_control.cpp`、`multiple_joint_angle_control.cpp`、`joint_enable_control.cpp`、`arm_zero_control.cpp` 和抓取示例向 `rt/arm_Command` 发布 `unitree_arm::msg::dds_::ArmString_`。
- `get_arm_joint_angle.cpp` 订阅 `current_servo_angle` 和 `arm_Feedback`。
- `d1_servo_heat_diagnosis.cpp` 订阅 `current_servo_angle`、`rt/current_servo_angle`、`arm_Feedback`、`rt/arm_Feedback`。
- `d1_joint5_control_diagnosis.cpp` 同时订阅上述反馈并可向 `rt/arm_Command` 发送受限诊断命令。

这些是可参考或单独运行的 vendored 工具，当前没有被
`rk_arm_control/rk_arm_control/d1_pick_node.py` 的
`UnitreeD1SdkAdapter` 调用。

## 5. Go2 Motion Adapter 和 Actions

### 5.1 当前调用拓扑

```text
/mission/run
  -> mission_state_machine_node
  -> /locomotion/execute_motion (rk_interfaces/ExecuteMotion)
  -> gait_control_node
  -> RobotMotionAdapter
  -> /navigation/cmd_vel
     -> 路径 A: cmd_vel_bridge_node -> /api/sport/request
     -> 路径 B: cmd_vel_udp_forwarder -> 外部 UDP server -> SportClient.Move()
```

直接 SDK 内置动作是另一条旁路：

```text
obstacle_direct_route_node / keyboard_route_node / sign_action_executor_node
  -> go2_sdk_motion_action
  -> Unitree SDK2 SportClient
```

该旁路没有统一接入 `/locomotion/execute_motion`。

### 5.2 指定 Action 审计

| Action | 上层调用 | Action server 映射 | 实际输出 | 判定 |
|---|---|---|---|---|
| `start_jump` | 有 | `JUMP_START_OBSTACLE` | 两段 `vx` 定时输出，中间 stop | 部分实现 |
| `finish_jump` | 有 | `JUMP_END_OBSTACLE` | 与 `start_jump` 相同的两段 `vx` 定时输出 | 部分实现 |
| `stairs_up_down` | 有 | 无 | goal 将返回 `unsupported motion_name` | 仅声明 |
| `avoid_zone` | 有 | `PRACTICAL_OBSTACLE_ZONE` | 多段前进/转向，融合可用的 depth/scan 最小距离 | 部分实现 |

关键风险：

- `start_jump` 和 `finish_jump` 没有调用已经存在的
  `SportClient.FrontJump()`。
- `prepare_obstacle_pose()` 明确保留 body height/gait mode TODO。
- `is_robot_stable()` 固定返回 `True`，roll/pitch 限制目前不生效。
- `RobotMotionAdapter.move()` 发布了 `linear.y`，但
  `Go2MotionClient` 的 Sport Move 参数把 `y` 固定为 0；只有 UDP 路径会转发
  `vy`。
- Action 的完成主要依据定时序列执行结束，不是机器人状态或足端接触反馈。

## 6. 机械臂接口

### 6.1 应用层接口

| 接口 | 类型 | 方向 | 当前用途 |
|---|---|---|---|
| `/arm/execute_task` | `rk_interfaces/action/ExecuteArmTask` | 任务状态机 -> arm node | 执行抓取、放置、回零等任务 |
| `/arm/command_json` | `std_msgs/msg/String` | 调试/上层 -> arm node | JSON 任务命令 |
| `/arm/status` | `std_msgs/msg/String` | arm node -> 上层 | JSON 任务状态 |
| `/arm/control_lock` | `std_msgs/msg/Bool` | arm node -> 上层 | 机械臂占用锁 |

### 6.2 Arm adapter

- 通用接口：`src/rk_arm_control/rk_arm_control/adapters/base.py`
  - `initialize`
  - `move_joints`
  - `open_gripper`
  - `close_gripper`
  - `stop`
  - `shutdown`
- 新机械臂实现：
  - `DryRunArmAdapter`：只模拟。
  - `SdkBridgeArmAdapter`：将 `MOVE_JOINTS`、`GRIPPER`、`STOP` 编码为 JSON。
- D1 实现：
  - `DryRunD1ArmAdapter`：只模拟。
  - `UnitreeD1SdkAdapter`：能定位 SDK 源码并做 pose table 查询，但真实 joint command、gripper 和 stop 都未实现。

### 6.3 SDK bridge

| 接口 | 类型 | 方向 | 当前状态 |
|---|---|---|---|
| `/arm/sdk_bridge/command_json` | `std_msgs/msg/String` | `SdkBridgeArmAdapter` -> bridge | 已实现发布和校验 |
| `/arm/sdk_bridge/status` | `std_msgs/msg/String` | bridge -> 观察者 | mock 可发布 `RECEIVED`、`DONE`、`FAILED` |

`new_arm_sdk_bridge_node.py` 支持 `mock` 和 `real` 两种模式，但：

- 默认配置为 `bridge_mode: mock`。
- `real.enable_real_sdk` 默认为 `false`。
- `_real_move_joints()`、`_real_gripper()`、`_real_stop()` 都会抛出
  `BridgeCommandError`，没有厂家 SDK 调用。
- `SdkBridgeArmAdapter` 不订阅 `/arm/sdk_bridge/status`。上层
  `fire_and_wait` 只按 `duration_sec` 等待，因此 arm Action 可能在硬件未确认时报告成功。

### 6.4 D1 DDS command/status

| 接口 | DDS 类型 | 方向 | 项目接入状态 |
|---|---|---|---|
| `rt/arm_Command` | `unitree_arm::msg::dds_::ArmString_` | 命令 | vendored 示例可发布；arm adapter 未接入 |
| `current_servo_angle` | `PubServoInfo_` | 状态 | 诊断工具可订阅；arm task 未接入 |
| `rt/current_servo_angle` | `PubServoInfo_` | 状态 | 诊断工具可订阅；arm task 未接入 |
| `arm_Feedback` | `ArmString_` | 状态 | 诊断工具可订阅；arm task 未接入 |
| `rt/arm_Feedback` | `ArmString_` | 状态 | 诊断工具可订阅；arm task 未接入 |

## 7. 雷达和深度接口

### 7.1 LaserScan

- 消费者：
  `gait_control_node.py` 可选订阅 `/scan`
  (`sensor_msgs/msg/LaserScan`)。
- 处理：
  按前、左、右角度窗口过滤有效量程，再取配置百分位值，并与 depth
  距离取最小值。
- 默认配置：
  `obstacle_safety.enable_scan: false`。
- 缺口：
  仓库内没有雷达 driver、SLAM launch、PointCloud-to-LaserScan 节点或任何
  `/scan` 发布者。

结论：**LaserScan 消费接口部分实现，硬件数据入口缺失。**

### 7.2 PointCloud

- 项目自有 `src/` 中没有 `sensor_msgs/msg/PointCloud2` 或
  `sensor_msgs/msg/PointCloud` 的 import、订阅、发布和处理。
- vendored `third_party/unitree_ros2/README.md` 说明 Go2 雷达点云 topic 为
  `/utlidar/cloud`，frame 为 `utlidar_lidar`。
- vendored `unitree_sdk2` 中存在 `PointCloud2_` 和 `LidarState_` DDS 类型，
  但这只是 SDK 类型支持，不等于项目已接入。

结论：**PointCloud 硬件能力在线索层面存在，在本项目中未集成。**

### 7.3 Depth image

| 项目 | 当前实现 |
|---|---|
| 驱动启动 | `rk_bringup/launch/realsense_low_bandwidth.launch.py` 和多个 bringup launch 可启动 `realsense2_camera` |
| 默认 topic | `/camera/camera/depth/image_rect_raw` |
| 消息类型 | `sensor_msgs/msg/Image` |
| 支持编码 | `16UC1`、`MONO16`、`32FC1` |
| 运动接入 | `gait_control_node.py` 提取左/前/右 ROI 距离并参与 `avoid_zone` 安全限制 |
| 诊断工具 | `rk_tools/depth_wall_distance_node.py` |
| 默认状态 | depth 开启，但 `require_fresh_data: false` |

结论：**Depth image 软件链路已实现，仍需真机校准和 fail-closed 验收。**

## 8. 缺失接口

### 8.1 Go2

1. `/locomotion/execute_motion` 到直接 `SportClient` 动作的统一 adapter。
2. `stairs_up_down` 的执行器、参数、安全停止和恢复流程。
3. Go2 low state/high state/IMU/足端接触订阅。
4. 基于真实姿态的稳定性判断和动作完成确认。
5. ROS 2 Sport Request 或直接 SDK 的侧向 `vy` 一致支持。
6. body height、gait mode、recovery stand 在 `RobotMotionAdapter` 中的真实实现。
7. 仓库内可复现的 UDP server，或对外部 server 的版本、源码和协议固定。

### 8.2 机械臂

1. 新机械臂真实厂家 SDK 的初始化、关节运动、夹爪和安全停止。
2. bridge status 到 `SdkBridgeArmAdapter`/arm Action 的反馈关联。
3. command `seq` 的 ACK、超时、拒绝、取消和去重语义。
4. D1 `rt/arm_Command` 在 `UnitreeD1SdkAdapter` 中的真实发布。
5. D1 gripper 和 emergency stop。
6. D1 关节反馈、动作完成、过热/故障状态到 `/arm/status` 的映射。
7. 已校准 IK 或固定点位 pose table；当前 pose table 为空。

### 8.3 雷达和深度

1. Go2 雷达/SLAM 驱动的项目级 launch。
2. `/utlidar/cloud` 的项目订阅和 frame/时间戳检查。
3. 若继续使用 `/scan`，缺少 PointCloud-to-LaserScan 或原生 LaserScan 生产者。
4. PointCloud/scan 与 D435i depth 的统一 frame 和 TF。
5. 传感器 stale/missing 时默认 fail-closed 的部署配置。
6. 深度相机安装外参、有效 ROI、最小/最大安全距离的真机标定记录。

## 9. TODO 列表

### P0：真机动作前必须完成

- [ ] 明确 Go2 生产控制路径：直接 SDK2、ROS 2 Sport Request、UDP 三选一作为主路径，其余只保留诊断或 fallback。
- [ ] 将 `start_jump` 和 `finish_jump` 接到经验证的跳跃后端；起点和终点若参数不同，使用独立配置。
- [ ] 实现或显式禁用 `stairs_up_down`，不得让 mission 运行到该阶段后才发现 unsupported goal。
- [ ] 接入 Go2 IMU/姿态状态，替换 `is_robot_stable() -> True`。
- [ ] 实现动作超时后的可靠 `StopMove`/恢复站立，并验证遥控器急停优先级。
- [ ] 实现真实机械臂 SDK bridge 的 move、gripper、stop。
- [ ] 让 arm Action 等待 bridge ACK/DONE，而不是只按 duration 睡眠。
- [ ] 确认雷达实际输出是 `/utlidar/cloud`、`/scan` 或其他 topic，并固定消息类型、frame 和频率。
- [ ] 生产配置启用传感器 freshness 检查；缺失/过期数据时停止而不是继续动作。

### P1：联调阶段完成

- [ ] 将外部 `go2_sdk_udp_server` 纳入版本管理，或移除该不可复现依赖。
- [ ] 补齐 `vy` 在所有 Go2 backend 中的一致语义和限速。
- [ ] 为 jump、stairs、avoid 建立统一的前置状态、执行状态、完成状态和恢复状态。
- [ ] 接入 `/utlidar/cloud`；若算法继续消费 `/scan`，增加受控的
  cloud-to-scan 转换和 TF 校验。
- [ ] 标定 D435i ROI、量程、安装姿态和障碍阈值。
- [ ] 完成机械臂关节数、角度单位、软限位、固定点位和夹爪力度标定。
- [ ] 把机械臂底层故障和取消结果传播到 `/arm/status` 与
  `ExecuteArmTask.Result`。

### P2：工程化

- [ ] 移除生产 launch 和节点中的用户目录硬编码路径，改为 package share、
  环境变量或显式参数。
- [ ] 为硬件 topic、DDS domain、网卡、RMW 和 SDK 版本建立单一部署清单。
- [ ] 增加 rosbag 回放测试：depth、LaserScan/PointCloud、Go2 state、arm feedback。
- [ ] 增加硬件在环测试记录模板，保留固件、SDK、网络、参数和测试结果。
- [ ] 对多个可控制 Go2/机械臂的节点增加互斥策略，防止并发命令源。

## 10. 需要人工测试的项目

所有运动测试都应在空旷、低速、有人持遥控器急停的条件下进行。机械臂主动测试前先完成无负载、软限位和温升检查。

| ID | 测试项目 | 人工验收点 |
|---|---|---|
| HW-GO2-01 | SDK2 网络发现 | 指定网卡可初始化；错误网卡明确失败；无 DDS/RMW 冲突 |
| HW-GO2-02 | `Move`/`StopMove` | 低速前进、转向方向正确；超时和节点退出均能停止 |
| HW-GO2-03 | `/api/sport/request` | API ID 和参数被机器人接受；与直接 SDK 行为一致 |
| HW-GO2-04 | UDP bridge | 外部 server 版本可追溯；`vx/vy/yaw`、watchdog、断网停止有效 |
| HW-GO2-05 | `go2_sdk_motion_action front_jump` | 跳前站稳、动作返回码、落地姿态、恢复和急停有效 |
| HW-GO2-06 | `start_jump` | 确认当前两段速度序列是否真的越障；未改后端前不得按名称假定会跳 |
| HW-GO2-07 | `finish_jump` | 与起点分别标定；确认是否需要不同距离、等待和恢复 |
| HW-GO2-08 | `stairs_up_down` | 当前应做负向测试并确认 goal 被拒绝；实现后再逐级、系绳测试 |
| HW-GO2-09 | `avoid_zone` + depth | 障碍距离、左右方向、减速、停止、stale image 行为正确 |
| HW-GO2-10 | Go2 姿态反馈 | roll/pitch 符号、单位、频率、stale 检测和翻倒停止正确 |
| HW-ARM-01 | bridge mock | command JSON、`seq`、RECEIVED/DONE/FAILED 状态格式正确 |
| HW-ARM-02 | D1 被动反馈 | 不发命令时能稳定收到 servo angle 和 arm feedback |
| HW-ARM-03 | D1 受限单关节诊断 | 冷机、无负载、小角度验证 joint ID；发现发热或错轴立即停止 |
| HW-ARM-04 | 新机械臂关节运动 | 每轴方向、零位、单位、软限位、超时和 ACK 正确 |
| HW-ARM-05 | 夹爪 | open/close 方向、力度、堵转、超时和释放策略正确 |
| HW-ARM-06 | arm stop/cancel | 任务取消、bridge 断开、节点退出时机械臂安全停止 |
| HW-ARM-07 | 完整 fixed-pose 任务 | 抓取/放置点位、碰撞间隙、负载、重复精度和温升合格 |
| HW-SENSOR-01 | D435i depth | topic、编码、帧率、量程、ROI 和实际测距误差合格 |
| HW-SENSOR-02 | Unitree PointCloud | `/utlidar/cloud` 是否存在；类型、frame、频率和点云朝向正确 |
| HW-SENSOR-03 | LaserScan | `/scan` 的真实生产链路、角度零点、左右方向、量程和频率正确 |
| HW-SENSOR-04 | 传感器失效 | 拔线、暂停 topic、旧时间戳时 `avoid_zone` 能 fail-closed |
| HW-SYSTEM-01 | 多命令源互斥 | mission、键盘、direct route、sign executor 不会同时控制机器人 |

## 11. 下一步开发建议

1. **先冻结硬件契约。** 确认 Go2 主控制 backend、雷达原始 topic、新机械臂型号及 SDK，再固定消息类型、topic、frame、网卡和错误语义。
2. **先闭合 Go2 安全链。** 接入 IMU/状态反馈，统一 stop/watchdog，再处理 jump 和 stairs；不要继续以定时结束代替动作完成。
3. **把雷达接入点一次定清。** 若设备原生输出 `/utlidar/cloud`，优先保留 PointCloud2 为原始数据，并只在确实需要时生成 `/scan`。
4. **完成机械臂 bridge 闭环。** 厂家 SDK 只放在 bridge 内，上层 adapter 通过 `seq + ACK/DONE/FAILED` 等待真实结果，取消和急停必须贯通。
5. **按风险递增做真机验收。** 被动订阅 -> 空载单轴/低速移动 -> 单动作 -> 组合动作 -> 任务链；每一步保存版本、参数和结果。
