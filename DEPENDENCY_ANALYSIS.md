# RK Inspection ROS2 - 模块依赖分析与集成指南

## 1. 包级依赖图 (Package Dependency Graph)

### 依赖关系矩阵

```
                  ↓依赖于→
           rk_i rk_c rk_co rk_b rk_p rk_m rk_n rk_t rk_u
rk_interfaces    -   -   -   -   -   -   -   -   -
rk_common        -   -   -   -   -   -   -   -   -
rk_config        -   -   -   -   -   -   -   -   -
rk_bringup       ✓   -   -   -   ✓   ✓   ✓   ✓   -
rk_perception    ✓   -   -   -   -   -   -   -   -
rk_mission       ✓   -   -   -   -   -   -   -   -
rk_navigation    ✓   -   -   -   -   -   -   -   -
rk_tools         ✓   -   -   -   -   -   -   -   -
rk_unitree_dr    -   -   -   -   -   -   -   -   -

图例: ✓ = 有依赖, - = 无依赖
列: rk_i=interfaces, rk_c=common, rk_co=config, rk_b=bringup, 
    rk_p=perception, rk_m=mission, rk_n=navigation, rk_t=tools, 
    rk_u=unitree_driver
```

### 详细依赖声明

```
rk_interfaces
├─ 编译依赖: rosidl_default_generators
├─ 运行依赖: std_msgs, geometry_msgs
└─ 其他: ament_cmake (C++包)

rk_common
├─ 编译依赖: ament_python
└─ 运行依赖: rclpy

rk_config
└─ 仅配置文件 (无代码依赖)

rk_perception
├─ 编译依赖: ament_python
├─ 运行依赖: rclpy, rk_interfaces, std_msgs, geometry_msgs
└─ 节点: 3个 (感知发布者)

rk_navigation
├─ 编译依赖: ament_python
├─ 运行依赖: rclpy, rk_interfaces, geometry_msgs
└─ 节点: 1个 (处理节点)

rk_mission
├─ 编译依赖: ament_python
├─ 运行依赖: rclpy, rk_interfaces
└─ 节点: 1个 (状态机)

rk_tools
├─ 编译依赖: ament_python
├─ 运行依赖: rclpy, rk_interfaces, std_srvs, geometry_msgs
└─ 节点: 5个 (服务器+工具)

rk_unitree_driver
├─ 编译依赖: ament_python
├─ 运行依赖: rclpy, geometry_msgs, rcl_interfaces, ament_index_python
└─ 节点: 1个 (桥接节点)
└─ 可选: unitree_api (后端=unitree_ros2时)

rk_bringup
├─ 编译依赖: ament_python
├─ 运行依赖: launch, launch_ros
├─ 软包依赖: rk_perception, rk_navigation, rk_tools, rk_mission
└─ Launch文件: 1个 (启动所有节点)
```

---

## 2. 消息依赖关系 (Message Dependencies)

### 自定义消息发布-订阅图

```
发布者 → 消息类型 → 订阅者
───────────────────────────────

mock_line_tracker_node
└─→ LineTrack
    └─→ line_follower_node

mock_sign_detector_node
└─→ SignDetectionArray
    └─→ (mission_state_machine可选监听)

mock_item_tag_node
└─→ ItemTagArray
    └─→ (mission_state_machine可选监听)

line_follower_node
└─→ Twist (geometry_msgs/Twist)
    └─→ cmd_vel_bridge_node

cmd_vel_bridge_node
└─→ Request (unitree_api/msg/Request)
    └─→ Unitree Go2 Sport (外部系统)
```

### 消息格式兼容性检查

```
✓ LineTrack
  ├─ std_msgs/Header: 标准ROS2格式
  ├─ float32 fields: 与C++兼容
  └─ 没有高级类型依赖

✓ SignDetection & SignDetectionArray
  ├─ 使用数组封装模式
  └─ 标准字符串和浮点类型

✓ ItemTag & ItemTagArray
  ├─ 包含 geometry_msgs/Pose
  ├─ 需要geometry_msgs库
  └─ 标准兼容格式

✓ Twist (标准ROS2)
  ├─ 来自 geometry_msgs
  ├─ 广泛支持
  └─ 所有ROS2系统兼容
```

---

## 3. 动作依赖关系 (Action Dependencies)

### 动作客户端-服务器配对

```
                Client Side                      Server Side
                ────────────────────────────────────────────

mission_state_machine_node           ←→    mock_locomotion_server
(ExecuteMotion Action Client)               (ExecuteMotion Action Server)
/mission/run 发送目标                        /locomotion/execute_motion
    ├─ motion_name (in)                        └─ 执行运动，返回结果
    ├─ current_step (feedback)                    ├─ 进度反馈
    └─ success (result)                          └─ 成功/失败

mission_state_machine_node           ←→    mock_arm_server
(ExecuteArmTask Action Client)              (ExecuteArmTask Action Server)
/mission/run 发送目标                        /arm/execute_task
    ├─ task_name, target (in)                    └─ 执行任务，返回结果
    ├─ current_step (feedback)                      ├─ 进度反馈
    └─ success (result)                           └─ 成功/失败

mission_client_node              ←→    mission_state_machine_node
(RunMission Action Client)               (RunMission Action Server)
/mission/run 手动触发                     /mission/run
    ├─ start (in)                           └─ 执行完整任务
    ├─ stage, progress (feedback)              ├─ 阶段反馈
    └─ success (result)                       └─ 最终结果

(同时)
mission_state_machine_node       ←→    mission_state_machine_node
(RunMission Action Client)              (RunMission Action Server)
自动自启动                             自动处理
    ├─ 1秒后触发                           └─ 接受并执行
    └─ 由 auto_start 参数控制
```

---

## 4. 参数依赖关系 (Parameter Dependencies)

### 参数声明与使用

```
节点: cmd_vel_bridge_node
声明的参数:
├─ backend (string, default="mock")
│  └─ 影响: Go2MotionClient 初始化
│  └─ 依赖于: unitree_ros2/unitree_api (if backend=unitree_ros2)
├─ cmd_vel_topic (string)
│  └─ 影响: 订阅话题
├─ sport_request_topic (string)
│  └─ 影响: 发布话题
├─ max_linear_x (double, default=0.20)
│  └─ 影响: SafetyMonitor 初始化
├─ max_angular_z (double, default=0.60)
│  └─ 影响: SafetyMonitor 初始化
├─ cmd_timeout_sec (double, default=0.50)
│  └─ 影响: Watchdog 定时器
├─ stop_publish_count (int, default=3)
│  └─ 影响: 停止命令重复
└─ stop_publish_period_sec (double, default=0.05)
   └─ 影响: 停止命令间隔

节点: line_follower_node
硬编码参数:
├─ confidence_threshold = 0.50
│  └─ 影响: LineTrack 消息过滤
├─ forward_speed = 0.20
│  └─ 影响: 输出速度幅度
├─ angular_gain = -1.20
│  └─ 影响: PID控制增益
└─ max_angular_speed = 0.60
   └─ 影响: 输出角速度钳制

节点: mission_state_machine_node
声明的参数:
└─ auto_start (bool, default=true)
   └─ 影响: 1秒后自动触发任务
   └─ 来源: launch文件参数
```

---

## 5. 时间同步依赖 (Time Synchronization)

### 时间戳使用情况

```
节点                          时间戳用途           精度要求
──────────────────────────────────────────────────────────

mock_line_tracker_node      消息header
  └─ get_clock().now()      0.5s周期             低（相对）

line_follower_node          消息处理              低（处理时标）

cmd_vel_bridge_node         日志记录              低（调试用）
  └─ 最后命令时间戳         超时检测             中（0.5s精度）
  └─ Watchdog timer        命令超时             中（50ms周期）

mission_state_machine_node  自动启动timer        低（1s精度）
  └─ 1秒延迟              

所有节点                    日志时间戳            低（调试用）

ROS2系统假设:
- 所有节点使用同一系统时钟
- 时间戳不需要外部同步（本地单机）
- 如需多机，需启用 ROS2 时间同步
```

---

## 6. 文件系统依赖 (File System Dependencies)

### 配置文件依赖

```
launch/mock_competition.launch.py
├─ 读取参数: auto_start
├─ 启动节点: 8个
└─ 隐含依赖:
   ├─ rk_perception (ament查找)
   ├─ rk_navigation (ament查找)
   ├─ rk_tools (ament查找)
   └─ rk_mission (ament查找)

launch/go2_cmd_vel_bridge.launch.py
├─ 读取配置文件: config/go2_driver.yaml
├─ 启动节点: cmd_vel_bridge_node
└─ 隐含依赖:
   └─ rk_unitree_driver (ament查找)

config/go2_driver.yaml
├─ 当前: 空文件
└─ 预期内容:
   └─ 可选的驱动参数 (当前使用代码默认值)
```

---

## 7. 硬件接口依赖 (Hardware Interface Dependencies)

### 外部硬件依赖

```
rk_unitree_driver
├─ 后端: mock
│  └─ 依赖: 无 (仅记录日志)
│  └─ 兼容性: 所有平台
│
└─ 后端: unitree_ros2
   ├─ 依赖: Unitree SDK v2.x
   ├─ 依赖: unitree_api ROS2 包
   ├─ 依赖: Go2 Sport 机器人硬件
   ├─ 兼容性: Linux + Unitree环境
   └─ 初始化: 需要 source_unitree_ros2.sh

外部话题 (如果后端=unitree_ros2):
└─ /api/sport/request
   ├─ 消息类型: unitree_api/msg/Request
   ├─ 使用: 发送运动命令到硬件
   └─ 依赖: Unitree ROS2 层
```

---

## 8. 运行时依赖关系 (Runtime Dependencies)

### 启动顺序分析

```
启动阶段 1: ROS2 Infrastructure
├─ ros2 daemon 启动
├─ DDS 发现启动
└─ 参数服务器初始化

启动阶段 2: Node Initialization (无强制顺序)
├─ rk_perception 节点
│  └─ 不依赖其他节点
│  └─ 立即开始发布数据
│
├─ rk_navigation 节点
│  └─ 可立即启动
│  └─ 等待 /perception/line_track 数据
│
├─ rk_tools 节点 (多个)
│  ├─ mock_locomotion_server
│  │  └─ 等待 /locomotion/execute_motion 调用
│  ├─ mock_arm_server
│  │  └─ 等待 /arm/execute_task 调用
│  └─ safety_node
│     └─ 等待 /safety/estop 调用
│
└─ rk_mission 节点
   ├─ 初始化后1秒
   ├─ 检查 auto_start 参数
   ├─ 如果 true: 自动发起任务
   └─ 等待其他服务器就绪 (通常已就绪)

启动阶段 3: Event Loop
├─ 所有节点进入 spin()
├─ 定时器启动
├─ 订阅回调准备就绪
└─ 动作服务器准备就绪

启动阶段 4: Autonomous Execution (if auto_start=true)
└─ mission_state_machine 1秒后自动触发任务
   ├─ 发送 goal_async()
   ├─ 等待其他服务器响应
   └─ 开始状态转移

关键依赖:
✓ 没有硬依赖 (所有包可独立启动)
✓ 逻辑依赖 (节点按需等待其他节点)
✓ 推荐启动: launch 文件 (确保所有节点就绪)
✓ 启动顺序: 任意 (因为有等待机制)
```

---

## 9. 集成清单 (Integration Checklist)

### 添加新模块的步骤

#### 场景 1: 添加新的感知源

```
1. 创建包 (rk_perception 内)
   rk_perception/rk_perception/new_sensor_node.py
   
2. 定义消息 (rk_interfaces 内)
   rk_interfaces/msg/NewSensorData.msg
   
3. 更新依赖
   rk_perception/package.xml
   └─ 添加 rk_interfaces 依赖
   
4. 更新 launch
   rk_bringup/launch/mock_competition.launch.py
   └─ 添加 Node(...) 条目

5. 验证
   colcon build
   ros2 launch rk_bringup mock_competition.launch.py
```

#### 场景 2: 添加新的执行器

```
1. 创建包 (rk_tools 内)
   rk_tools/rk_tools/new_server_node.py
   
2. 定义动作 (rk_interfaces 内)
   rk_interfaces/action/NewExecutor.action
   
3. 更新依赖
   rk_tools/package.xml
   └─ 添加 rk_interfaces 依赖
   
4. 修改状态机
   rk_mission/rk_mission/mission_state_machine_node.py
   └─ 添加新状态处理逻辑
   
5. 更新 launch
   rk_bringup/launch/mock_competition.launch.py
   └─ 添加新服务器节点

6. 验证
   colcon build
   ros2 launch rk_bringup mock_competition.launch.py
```

#### 场景 3: 添加新的控制逻辑

```
1. 选择现有节点或创建新节点

2. 订阅必要的话题
   └─ 在 __init__ 中创建订阅
   
3. 发布处理结果
   └─ 在 __init__ 中创建发布者
   
4. 实现回调函数
   └─ def on_message(self, msg)
   
5. 更新依赖和 launch

6. 测试消息流
   ros2 topic echo /your_topic
```

---

## 10. 故障排查指南 (Troubleshooting Guide)

### 依赖问题诊断

```
症状: 包找不到 (Package not found)
原因: 编译或安装问题
排查:
  1. colcon build 输出
  2. source install/setup.bash
  3. ros2 pkg list | grep rk_

症状: 消息类型不可用
原因: 接口包未编译或未生成
排查:
  1. 检查 rk_interfaces/msg/*.msg 文件
  2. colcon build rk_interfaces --packages-select
  3. 检查 package.xml 中的依赖声明

症状: 动作服务不响应
原因: 服务器未启动或未就绪
排查:
  1. ros2 action list
  2. ros2 node list
  3. 检查 launch 文件中是否启动了服务器

症状: 参数无效
原因: 类型不匹配或参数名错误
排查:
  1. ros2 param list /node_name
  2. ros2 param get /node_name param_name
  3. 检查参数类型 (double vs int vs string)

症状: 消息丢失或延迟
原因: 缓冲区不足或节点不够快
排查:
  1. ros2 topic hz /topic_name
  2. ros2 topic bw /topic_name
  3. 检查发布频率 vs 处理时间
```

---

## 11. 依赖关系最小化树 (Minimal Dependency Tree)

### 最简化配置

```
要运行最小的导航系统，仅需:

rk_interfaces (基础)
├─ LineTrack.msg
├─ ExecuteMotion.action
└─ ExecuteArmTask.action

rk_perception (感知)
└─ mock_line_tracker_node
   └─ 发布 /perception/line_track

rk_navigation (导航)
└─ line_follower_node
   ├─ 订阅 /perception/line_track
   └─ 发布 /navigation/cmd_vel

rk_unitree_driver (执行)
└─ cmd_vel_bridge_node
   ├─ 订阅 /navigation/cmd_vel
   └─ 发布/记录 API 请求

总计: 4个包，完整的感知→控制→执行链

添加任务管理:
+rk_mission (协调)
└─ mission_state_machine_node

+rk_tools (执行器)
├─ mock_locomotion_server
└─ mock_arm_server

总计: 6个包，完整的竞赛系统
```

---

## 12. 扩展性分析 (Scalability Analysis)

### 添加新组件的复杂度

```
组件类型          实现复杂度    集成复杂度    建议
─────────────────────────────────────────
新感知节点        低           低           ✓ 容易添加
新导航算法        低-中         低           ✓ 推荐
新执行器/服务    中            中           ✓ 可行
新状态/阶段      低            中           ✓ 可行
新消息类型        低            中           ✓ 必要时添加
新动作类型        中            高           ⚠ 需要协调
硬件集成          高            高           ⚠ 复杂
```

---

**生成时间**: 2026-05-09  
**版本**: 1.0  
**适用范围**: RK Inspection ROS2 工作区
