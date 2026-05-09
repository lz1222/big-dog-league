# RK Inspection ROS2 - 快速参考指南

## 系统概览 (一页纸总结)

### 项目结构
```
Perception (感知)          →  Processing (处理)      →  Execution (执行)
├─ Line Tracker            ├─ Line Follower          ├─ Cmd Vel Bridge
├─ Sign Detector    ────→  └─ [PID Control]    ────→ ├─ Go2 Motion Client
└─ Item Tag                                          ├─ Safety Monitor
                                                     └─ Mock/Real Hardware
         ↓                                                    ↓
    Perception/               Navigation/                  API/Sport
    line_track,               cmd_vel                      request
    sign_detections,                                       (JSON/Mock)
    item_tags
                           ↑─ Mission State Machine ←─
                           Coordinates all stages
```

### 核心路径 (3条主要数据流)

#### 1. **视觉导航路径** (感知 → 控制 → 执行)
```
LineTracker
(lateral_error)
    ↓
LineFollower
(PID计算)
    ↓
CmdVel
(Twist: 0.2 m/s, ±0.6 rad/s)
    ↓
CmdVelBridge
(安全检查)
    ↓
Go2 Motion
(API请求)
```

#### 2. **任务协调路径** (状态机 → 执行器)
```
MissionStateMachine
(12阶段状态机)
    ├─ LOCOMOTION ───→ MockLocomotionServer
    │                  (腿部运动)
    │
    └─ ARM_TASK ────→ MockArmServer
                      (机械臂任务)
```

#### 3. **感知发布路径** (多源感知)
```
感知节点（独立运行）
├─ mock_line_tracker_node     (0.5s)
├─ mock_sign_detector_node    (1.0s)
└─ mock_item_tag_node         (1.0s)
```

---

## 关键节点速查表

| 节点 | 包 | 输入 | 输出 | 功能 |
|------|------|------|------|------|
| mock_line_tracker | rk_perception | - | `/perception/line_track` | 发布线路跟踪数据 |
| line_follower | rk_navigation | `/perception/line_track` | `/navigation/cmd_vel` | 转换为速度命令 |
| cmd_vel_bridge | rk_unitree_driver | `/navigation/cmd_vel` | `/api/sport/request` | 安全检查+API转换 |
| mission_state_machine | rk_mission | `/mission/run (action)` | `/locomotion/*`, `/arm/*` (action) | 12阶段任务管理 |
| mock_locomotion_server | rk_tools | `/locomotion/execute_motion` | result+feedback | 模拟腿部运动 |
| mock_arm_server | rk_tools | `/arm/execute_task` | result+feedback | 模拟机械臂 |
| safety_node | rk_tools | `/safety/estop (service)` | - | 紧急停止服务 |

---

## 消息类型 (数据结构)

```
LineTrack
├─ header: Header
├─ lateral_error: float32     (-0.04 ~ 0.04)
├─ heading_error: float32     (±0.02)
├─ confidence: float32        (0.95)
└─ line_visible: bool         (true/false)

Twist (ROS2标准)
├─ linear.x: float64          (0.2 m/s)
├─ linear.y: float64          (0)
├─ linear.z: float64          (0)
├─ angular.x: float64         (0)
├─ angular.y: float64         (0)
└─ angular.z: float64         (-0.6 ~ 0.6 rad/s)

ExecuteMotion Action
├─ Request:  motion_name (string)
├─ Result:   success (bool), message (string)
└─ Feedback: current_step (string), progress (0.0-1.0)
```

---

## 安全限制 (Safety Limits)

| 参数 | 限制值 | 检查点 |
|------|-------|--------|
| max_linear_x | 0.20 m/s | CmdVelBridge + SafetyMonitor |
| max_angular_z | 0.60 rad/s | CmdVelBridge + SafetyMonitor |
| cmd_timeout | 0.50 sec | CmdVelBridge Watchdog |
| confidence_threshold | 0.50 | LineFollower |
| stop_republish_count | 3 times | CmdVelBridge |

---

## 参数改动清单

### 如何调整导航参数

**修改文件**: `src/rk_navigation/rk_navigation/line_follower_node.py`
```python
self.confidence_threshold = 0.50    # 改为更严格 0.70
self.forward_speed = 0.20           # 改为更快 0.30
self.angular_gain = -1.20           # 改为更敏感 -2.0
self.max_angular_speed = 0.60       # 改为更大 1.0
```

### 如何调整速度限制

**方法1 - 修改代码** (`src/rk_unitree_driver/rk_unitree_driver/cmd_vel_bridge_node.py`):
```python
self.declare_parameter('max_linear_x', 0.20)
self.declare_parameter('max_angular_z', 0.60)
```

**方法2 - Launch参数** (推荐):
```bash
ros2 launch rk_bringup mock_competition.launch.py \
  max_linear_x:=0.30 max_angular_z:=0.80
```

**方法3 - 运行时参数更新**:
```bash
ros2 param set /cmd_vel_bridge_node max_linear_x 0.30
ros2 param set /cmd_vel_bridge_node max_angular_z 0.80
```

---

## 任务阶段流程 (12 Stages)

```
PRECHECK (预检查)
    ↓
START (启动) ──→ ExecuteMotion("start")
    ↓
LINE_FOLLOW (线路跟踪) ──→ ExecuteMotion("line_follow")
    ↓
AVOID (避障) ──→ ExecuteMotion("avoid")
    ↓
STAIRS (楼梯) ──→ ExecuteMotion("stairs")
    ↓
PICK_START_ITEM (拾取起始物品) ──→ ExecuteArmTask("pick_start_item", "start_item")
    ↓
TRANSFER_AND_PICK_FIELD_ITEM ──→ ExecuteArmTask("pick_field_item", "field_item")
    ↓
WARNING_DETECT_ACTION ──→ ExecuteMotion("warning_detect")
    ↓
PLACE_ITEM (放置物品) ──→ ExecuteArmTask("place_item", "finish_platform")
    ↓
JUMP_FINISH (跳跃终点) ──→ ExecuteMotion("jump_finish")
    ↓
FINAL_STOP (最终停止) ──→ ExecuteMotion("final_stop")
    ↓
DONE (完成) ✓
```

---

## 调试命令速查

```bash
# 基础监控
ros2 topic list                        # 列出所有话题
ros2 topic echo /perception/line_track # 监听线路跟踪
ros2 action list                       # 列出所有动作
ros2 node list                         # 列出所有节点

# 实时监听
ros2 topic hz /navigation/cmd_vel      # 监测速度命令频率
ros2 topic bw /navigation/cmd_vel      # 监测带宽

# 调试任务执行
ros2 action send_goal /mission/run rk_interfaces/action/RunMission "{start: true}"
ros2 run rk_tools mission_client_node  # 手动启动

# 测试速度命令
ros2 run rk_tools two_step_walk_test_node

# 服务调用
ros2 service call /safety/estop std_srvs/SetBool "{data: true}"   # 紧急停止
ros2 service call /safety/estop std_srvs/SetBool "{data: false}"  # 释放停止

# 参数查询
ros2 param list /cmd_vel_bridge_node
ros2 param get /cmd_vel_bridge_node max_linear_x

# 日志级别调整
ros2 launch rk_bringup mock_competition.launch.py --log-level DEBUG
```

---

## 常见问题解决 (Troubleshooting)

| 问题 | 症状 | 解决 |
|------|------|------|
| 机器人不运动 | cmd_vel被拒绝 | 检查安全限制、检查confidence |
| 命令频繁停止 | 看门狗超时 | 增加cmd_timeout_sec 或检查感知数据 |
| 旋转过度 | 角速度太大 | 减小angular_gain 或 max_angular_speed |
| 任务卡住 | 某阶段无进度 | 检查动作服务器、查看日志 |
| 无法连接硬件 | unitree_ros2后端失败 | source Unitree workspace, 检查ROS_DOMAIN_ID |

---

## 性能指标 (Performance)

| 指标 | 值 | 说明 |
|------|-----|------|
| 感知延迟 | 0.5-1.0s | mock节点发布周期 |
| 控制延迟 | <50ms | 感知→命令转换时间 |
| 命令响应 | <100ms | 从cmd_vel到硬件API |
| 任务周期 | ~12s | 完整竞赛12阶段（每阶段1s） |
| CPU占用 | <5% | mock系统 (实际系统会更高) |

---

## 文件修改指南

### 添加新的感知源
**文件**: `src/rk_perception/rk_perception/new_sensor_node.py`
```python
class NewSensorNode(Node):
    def __init__(self):
        super().__init__('new_sensor_node')
        self.publisher = self.create_publisher(CustomMsg, '/perception/new_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
    
    def publish_data(self):
        msg = CustomMsg()
        # 填充消息
        self.publisher.publish(msg)
```
然后在 `mock_competition.launch.py` 中添加节点。

### 添加新的执行器
**文件**: `src/rk_tools/rk_tools/new_server.py`
```python
class NewExecutor(Node):
    def __init__(self):
        super().__init__('new_executor')
        self.action_server = ActionServer(
            self, CustomAction, '/new/execute',
            self.execute_callback
        )
```

### 添加新的任务阶段
**文件**: `src/rk_mission/rk_mission/mission_state_machine_node.py`
```python
STAGES = [
    # ... existing ...
    'NEW_STAGE',  # 添加
]

# 如果需要调用动作
if stage == 'NEW_STAGE':
    # 调用相应的服务器
```

---

## 第三方依赖

| 库 | 版本 | 来源 | 用途 |
|----|------|------|------|
| ROS2 | Humble | ros.org | 核心框架 |
| rclpy | 标准 | ROS2 | Python客户端 |
| Unitree SDK | v2.x | third_party/ | 硬件控制 |
| geometry_msgs | 标准 | ROS2 | 几何消息 |
| std_msgs | 标准 | ROS2 | 基础消息 |

---

## 构建和部署

### 编译步骤
```bash
cd ~/rk_inspection_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 验证构建
```bash
# 检查所有包
ros2 pkg list | grep rk_

# 检查特定节点
ros2 run rk_perception mock_line_tracker_node  # 应启动
```

### 部署到真实硬件
```bash
# 切换后端
ros2 launch rk_unitree_driver go2_cmd_vel_bridge.launch.py \
  backend:=unitree_ros2

# 或在mission启动时指定
ros2 launch rk_bringup mock_competition.launch.py \
  # (需要修改launch文件以支持backend参数)
```

---

**生成时间**: 2026-05-09  
**版本**: 1.0  
**维护者**: ZhenLi (2605128876@qq.com)
