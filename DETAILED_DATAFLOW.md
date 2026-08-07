# RK Inspection 系统 - 详细数据流分析

## 1. 完整系统拓扑 (System Topology)

```
┌────────────────────────────────────────────────────────────────────────────┐
│                          ROS2 Middleware (rclcpp/rclpy)                   │
└────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────┬──────────────────────────────┬──────────────┐
│       PERCEPTION LAYER       │      CONTROL LAYER          │  EXEC LAYER  │
├──────────────────────────────┼──────────────────────────────┼──────────────┤
│                              │                              │              │
│  ┌──────────────────────┐   │  ┌────────────────────────┐  │  ┌─────────┐ │
│  │ mock_line_tracker_   │   │  │ line_follower_node    │  │  │ cmd_vel │ │
│  │      node            │   │  │                        │  │  │ _bridge │ │
│  │                      │   │  │ Algorithm:             │  │  │  _node  │ │
│  │ Rate: 0.5s          │───┼──│ angular =              │  │  │         │ │
│  │ Pub: /perception/    │   │  │ -1.2 * lateral_error  │─┼──│         │ │
│  │      line_track      │   │  │                        │  │  │ Safety: │ │
│  │                      │   │  │ linear = 0.2 (const)  │  │  │ • Check │ │
│  │ Fields:              │   │  │                        │  │  │   valid │ │
│  │ • lateral_error      │   │  │ clamp(angular,        │  │  │ • Check │ │
│  │ • heading_error      │   │  │ -0.6, 0.6)            │  │  │   limit │ │
│  │ • confidence: 0.95   │   │  │                        │  │  │ • Check │ │
│  │ • line_visible: true │   │  │ Sub: /perception/     │  │  │   zeros │ │
│  └──────────────────────┘   │  │     line_track        │  │  │         │ │
│                              │  │ Pub: /navigation/     │  │  │ Backend │ │
│  ┌──────────────────────┐   │  │     cmd_vel           │──┼──│         │ │
│  │ mock_sign_detector_  │   │  │ Confidence Threshold: │  │  │ • mock  │ │
│  │      node            │   │  │ 0.50                  │  │  │ • real  │ │
│  │                      │   │  │                        │  │  │         │ │
│  │ Rate: 1.0s          │   │  └────────────────────────┘  │  │ Go2 API │ │
│  │ Pub: /perception/    │   │                              │  │ • MOVE: │ │
│  │      sign_detections │   │                              │  │   1008  │ │
│  │                      │   │                              │  │ • STOP: │ │
│  │ Types cycle:         │   │                              │  │   1003  │ │
│  │ • direction/forward  │   │                              │  │         │ │
│  │ • warning/caution    │   │                              │  │ Output: │ │
│  │ • task/pick_item     │   │                              │  │ /api/   │ │
│  └──────────────────────┘   │                              │  │ sport/  │ │
│                              │                              │  │ request │ │
│  ┌──────────────────────┐   │                              │  └─────────┘ │
│  │ mock_item_tag_node   │   │                              │              │
│  │                      │   │  ┌────────────────────────┐  │  ┌─────────┐ │
│  │ Rate: 1.0s          │   │  │ mission_state_machine_ │  │  │ mock_   │ │
│  │ Pub: /perception/    │───┼──│      node              │─┼──│ locomot │ │
│  │      item_tags       │   │  │                        │  │  │ ion_    │ │
│  │                      │   │  │ States (12):           │  │  │ server  │ │
│  │ Fixed Items:         │   │  │ 1. PRECHECK           │  │  │         │ │
│  │ 1. start_item        │   │  │ 2. START              │  │  │ Action: │ │
│  │    (0.8, 0.1, 0.2)   │   │  │ 3. LINE_FOLLOW        │  │  │ /locom  │ │
│  │ 2. field_item        │   │  │ 4. AVOID              │  │  │ otion/  │ │
│  │    (1.6,-0.2, 0.2)   │   │  │ 5. STAIRS             │  │  │ execute │ │
│  │                      │   │  │ 6. PICK_START_ITEM    │  │  │ _motion │ │
│  │ confidence: 0.90     │   │  │ 7. TRANSFER_PICK_     │  │  │         │ │
│  └──────────────────────┘   │  │    FIELD_ITEM         │  │  │ Input:  │ │
│                              │  │ 8. WARNING_DETECT     │  │  │ motion_ │ │
│                              │  │ 9. PLACE_ITEM        │  │  │ name    │ │
│  ┌──────────────────────┐   │  │ 10. JUMP_FINISH       │  │  │ Output: │ │
│  │ Safety Node          │   │  │ 11. FINAL_STOP        │  │  │ succes  │ │
│  │                      │   │  │ 12. DONE              │  │  │ prog    │ │
│  │ Service:             │   │  │                        │  │  └─────────┘ │
│  │ /safety/estop        │   │  │ Pub: /mission/run     │  │              │
│  │ SetBool Srv          │───┼──│ Sub: /locomotion/,    │  │  ┌─────────┐ │
│  │                      │   │  │      /arm/            │──┼──│ mock_   │ │
│  │ emergency stop       │   │  │                        │  │  │ arm_    │ │
│  │ (mock only)          │   │  │ Auto-trigger via:      │  │  │ server  │ │
│  └──────────────────────┘   │  │ Timer(1.0s) when      │  │  │         │ │
│                              │  │ auto_start=true        │  │  │ Action: │ │
│                              │  │                        │  │  │ /arm/   │ │
│  ┌──────────────────────┐   │  │ Reentrant callbacks    │  │  │ execute │ │
│  │ Mission Client Node  │   │  │ for parallel exec      │  │  │ _task   │ │
│  │                      │   │  │                        │  │  │         │ │
│  │ Action Client        │───┼──│ Invoke /locomotion/    │  │  │ Input:  │ │
│  │ /mission/run         │   │  │ execute_motion         │  │  │ task_   │ │
│  │                      │   │  │ Invoke /arm/execute_   │  │  │ name,   │ │
│  │ For manual trigger   │   │  │       task             │  │  │ target  │ │
│  │ (when auto_start:=   │   │  │                        │  │  │ Output: │ │
│  │  false)              │   │  │ Wait for results       │  │  │ succes  │ │
│  └──────────────────────┘   │  │ Handle feedback        │  │  │ prog    │ │
│                              │  └────────────────────────┘  │  └─────────┘ │
└──────────────────────────────┴──────────────────────────────┴──────────────┘
                                          │
                                          ▼
                        ┌─────────────────────────────┐
                        │  Unitree Go2 Sport Robot    │
                        │  (Mock Backend / Real HW)   │
                        │                             │
                        │ • Locomotion (腿部运动)    │
                        │ • D1 Arm (机械臂)         │
                        │ • D435i Camera (视觉)      │
                        └─────────────────────────────┘
```

---

## 2. 消息流时间序列 (Message Timing Diagram)

```
时间轴 (Timeline):
0s          1s          2s          3s          4s          5s

LineTracker
├─ 0.5s: Pub #1 ──┐
├─ 1.0s: Pub #2   │ ──────┬─────────┐
├─ 1.5s: Pub #3        │          │
├─ 2.0s: Pub #4        │          │
├─ 2.5s: Pub #5   ─────┤          │
├─ 3.0s: Pub #6        │     ┌────┤
├─ 3.5s: Pub #7   ─────┼─────┤    │
├─ 4.0s: Pub #8        │     │    │
├─ 4.5s: Pub #9   ─────┴─────┤    │
└─ 5.0s: Pub #10              │    │
                              ▼    ▼
LineFollower Callbacks
├─ ~0.5s: CB #1 ──→ Process ──→ Pub cmd_vel #1 
├─ ~1.0s: CB #2 ──→ Process ──→ Pub cmd_vel #2
├─ ~1.5s: CB #3 ──→ Process ──→ Pub cmd_vel #3
├─ ~2.0s: CB #4 ──→ Process ──→ Pub cmd_vel #4
├─ ~2.5s: CB #5 ──→ Process ──→ Pub cmd_vel #5
├─ ~3.0s: CB #6 ──→ Process ──→ Pub cmd_vel #6
├─ ~3.5s: CB #7 ──→ Process ──→ Pub cmd_vel #7
├─ ~4.0s: CB #8 ──→ Process ──→ Pub cmd_vel #8
├─ ~4.5s: CB #9 ──→ Process ──→ Pub cmd_vel #9
└─ ~5.0s: CB #10 ─→ Process ──→ Pub cmd_vel #10
                                │
                                ▼
CmdVelBridge Callbacks
├─ ~0.5s: CB #1 ──→ SafetyCheck ──→ Go2API.send_move()
├─ ~1.0s: CB #2 ──→ SafetyCheck ──→ Go2API.send_move()
├─ ~1.5s: CB #3 ──→ SafetyCheck ──→ Go2API.send_move()
└─ ...继续

SignDetector (独立, 1.0s周期)
├─ 1.0s: Pub #1 (direction/forward)
├─ 2.0s: Pub #2 (warning/caution)
├─ 3.0s: Pub #3 (task/pick_item)
├─ 4.0s: Pub #4 (direction/forward)
└─ 5.0s: Pub #5 (warning/caution)

ItemTag (独立, 1.0s周期)
├─ 1.0s: Pub #1 (2 items)
├─ 2.0s: Pub #2 (2 items)
├─ 3.0s: Pub #3 (2 items)
├─ 4.0s: Pub #4 (2 items)
└─ 5.0s: Pub #5 (2 items)

MissionStateMachine (初始化后1s)
├─ 1.0s: AutoStart → send_goal_async(/mission/run)
│          ├─ goal_callback() → GoalResponse.ACCEPT
│          └─ execute_callback():
│             ├─ Stage 0: PRECHECK (instant)
│             ├─ Stage 1: START 
│             │           ├─ send_goal_async(/locomotion/execute_motion)
│             │           ├─ wait → ~1s
│             │           └─ next_stage
│             ├─ Stage 2: LINE_FOLLOW
│             │           ├─ send_goal_async(/locomotion/execute_motion)
│             │           ├─ wait → ~1s
│             │           └─ next_stage
│             └─ ... (10 more stages)
```

---

## 3. 控制流算法 (Control Flow Algorithm)

### LineFollower 算法 (伪代码)

```pseudocode
每 LineTrack 消息到达:
  
  收到消息 msg: LineTrack
  
  IF NOT msg.line_visible:
    // 线路不可见，停止
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    发布 cmd_vel
    返回
  
  IF msg.confidence < 0.50:
    // 置信度不足，停止
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    发布 cmd_vel
    返回
  
  // 计算角速度 (PID-like)
  angular = angular_gain * msg.lateral_error
  angular = -1.20 * msg.lateral_error
  
  // 钳制在最大值
  IF angular > 0.60:
    angular = 0.60
  IF angular < -0.60:
    angular = -0.60
  
  // 设置线性速度 (常数)
  linear = 0.20  // m/s
  
  // 构建 Twist 消息
  cmd_vel.linear.x = linear
  cmd_vel.angular.z = angular
  
  发布 cmd_vel 到 /navigation/cmd_vel
```

### SafetyMonitor 验证 (伪代码)

```pseudocode
函数 evaluate(linear_x, angular_z) -> CommandDecision:
  
  vx = float(linear_x)
  vyaw = float(angular_z)
  
  // 检查1：有效性检查
  IF isnan(vx) OR isinf(vx):
    返回 CommandDecision(
      should_stop=True,
      reason="linear.x is invalid"
    )
  
  IF isnan(vyaw) OR isinf(vyaw):
    返回 CommandDecision(
      should_stop=True,
      reason="angular.z is invalid"
    )
  
  // 检查2：速度限制检查
  IF abs(vx) > max_linear_x:
    返回 CommandDecision(
      should_stop=True,
      reason=f"linear.x {vx} exceeds {max_linear_x}"
    )
  
  IF abs(vyaw) > max_angular_z:
    返回 CommandDecision(
      should_stop=True,
      reason=f"angular.z {vyaw} exceeds {max_angular_z}"
    )
  
  // 检查3：零速度检查 (可选策略)
  IF vx == 0.0 AND vyaw == 0.0:
    返回 CommandDecision(
      should_stop=True,
      reason="zero velocity command"
    )
  
  // 所有检查通过
  返回 CommandDecision(
    should_stop=False,
    reason="ok",
    vx=vx,
    vyaw=vyaw
  )
```

---

## 4. 状态机转移图 (State Machine)

```
                     ┌──────────────────────┐
                     │    系统启动           │
                     │ (launch文件启动)      │
                     └──────────┬───────────┘
                                │
                                ▼
                     ┌──────────────────────┐
                     │ 所有节点初始化完成   │
                     │ 包括定时器和服务器   │
                     └──────────┬───────────┘
                                │
                ┌───────────────┴───────────────┐
                │                               │
                ▼                               ▼
        ┌─────────────────┐           ┌──────────────────┐
        │ auto_start=true │           │ auto_start=false │
        └────────┬────────┘           └────────┬─────────┘
                 │                             │
                 ▼                             ▼
        ┌─────────────────────────────┐   ┌──────────────────┐
        │ Timer(1.0s) fired           │   │ 等待外部触发      │
        │ send_auto_goal to /mission  │   │ (mission_client) │
        │ /run                        │   └────────┬─────────┘
        └────────┬────────────────────┘           │
                 │                                 │
                 └─────────────────┬───────────────┘
                                   │
                                   ▼
                     ┌──────────────────────────┐
                     │ mission_state_machine    │
                     │ execute_callback()       │
                     │ 进入状态循环             │
                     └──────────┬───────────────┘
                                │
    ┌───────────────────────────┼───────────────────────────┐
    │                           │                           │
    ▼                           ▼                           ▼
State 0: PRECHECK      State 1-12: 任务执行      State 12: DONE
(immediate)            (conditional)              (terminal)
    │                           │                           │
    ├─ 检查系统       ┌─────────┴─────────────┐  ├─ 返回成功
    │   状态         │                       │  │ result
    │                │ Locomotion States     │  │
    │                │ (START, LINE_FOLLOW,  │  └──────────┐
    │                │  AVOID, STAIRS, etc)  │             │
    │                │ ───→ ExecuteMotion    │             │
    │                │      Action Server    │             │
    │                │      callback()       │             │
    │                │ ←─── return result    │             │
    │                │                       │             │
    │                │ Arm Execution States  │             │
    │                │ (PICK_*, PLACE_ITEM) │             │
    │                │ ───→ ExecuteArmTask   │             │
    │                │      Action Server    │             │
    │                │      callback()       │             │
    │                │ ←─── return result    │             │
    │                │                       │             │
    │                └─────────┬─────────────┘             │
    │                          │                          │
    │          ┌───────────────┴────────────────┐         │
    │          │                                │         │
    │          ▼                                ▼         │
    │      success                          failure       │
    │          │                                │         │
    └──────────┼────────────────────────────────┼─────────┘
               │                                │
               ├─ next_stage = (current + 1)   │ goal_handle.abort()
               │ current < DONE?                │ return (success=False)
               │ ├─ YES: continue loop         │
               │ └─ NO: break                  │
               │                               │
               └───────────────┬────────────────┘
                               │
                               ▼
                     ┌──────────────────────┐
                     │ goal_handle.succeed()│
                     │ 返回最终结果         │
                     │ (success=True)       │
                     └──────────┬───────────┘
                                │
                                ▼
                     ┌──────────────────────┐
                     │ mission_running=False│
                     │ 任务完成              │
                     └──────────────────────┘
```

---

## 5. 消息详细结构 (Detailed Message Structures)

### LineTrack 消息示例

```yaml
消息类型: rk_interfaces/msg/LineTrack
发布者: mock_line_tracker_node
订阅者: line_follower_node
发布频率: 0.5 秒

消息内容示例:
  header:
    stamp:
      sec: 1234567890
      nanosec: 500000000        # 0.5秒周期
    frame_id: "d435i_color_optical_frame"
  
  lateral_error: -0.04          # 范围: [-0.04, 0.04]
  heading_error: 0.02           # 范围: ±0.02
  confidence: 0.95              # 范围: [0.0, 1.0]
  line_visible: true            # 布尔值

解释:
  • lateral_error < 0: 偏向左边
  • lateral_error > 0: 偏向右边
  • 绝对值越大，偏差越严重
```

### Twist 消息示例

```yaml
消息类型: geometry_msgs/msg/Twist
发布者: line_follower_node
订阅者: cmd_vel_bridge_node
发布频率: 接收 LineTrack 频率 (~0.5s)

消息内容示例 (直线运动):
  linear:
    x: 0.2                       # m/s, 前进速度
    y: 0.0                       # m/s, 侧向速度 (未使用)
    z: 0.0                       # m/s, 上升速度 (未使用)
  
  angular:
    x: 0.0                       # rad/s (未使用)
    y: 0.0                       # rad/s (未使用)
    z: 0.048                     # rad/s, 旋转速度

消息内容示例 (停止):
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}

活跃字段:
  • linear.x: -0.2 ~ 0.2 (受限)
  • angular.z: -0.6 ~ 0.6 (受限)
```

### ExecuteMotion Action 请求/结果

```yaml
消息类型: rk_interfaces/action/ExecuteMotion

请求 (Goal):
  motion_name: "line_follow"     # 要执行的运动名称

结果 (Result) - 完成后返回:
  success: true                  # 是否成功
  message: "Motion completed: line_follow"

反馈 (Feedback) - 定期发布:
  current_step: "line_follow: step 2/5"
  progress: 0.4                  # 0.0 ~ 1.0, 每步 0.2
```

### ExecuteArmTask Action 请求/结果

```yaml
消息类型: rk_interfaces/action/ExecuteArmTask

请求 (Goal):
  task_name: "pick_field_item"   # 任务名称
  target: "field_item"            # 目标物品

结果 (Result) - 完成后返回:
  success: true
  message: "Arm task completed: pick_field_item -> field_item"

反馈 (Feedback) - 定期发布:
  current_step: "pick_field_item: step 3/5"
  progress: 0.6                  # 0.0 ~ 1.0
```

### RunMission Action 请求/结果

```yaml
消息类型: rk_interfaces/action/RunMission

请求 (Goal):
  start: true                    # 启动标志

结果 (Result) - 完成后返回:
  success: true
  message: "Mission completed successfully"

反馈 (Feedback) - 定期发布:
  stage: "LINE_FOLLOW"           # 当前阶段
  progress: 0.25                 # 完成进度 (1/12 ≈ 0.083, 3/12 = 0.25)
```

---

## 6. 关键参数范围表 (Parameter Reference)

```
┌─────────────────────────────────────────────────────────────┐
│ 导航参数 (rk_navigation/line_follower_node)                │
├──────────────────────┬────────────┬─────────┬──────────────┤
│ 参数名               │ 默认值     │ 范围    │ 说明         │
├──────────────────────┼────────────┼─────────┼──────────────┤
│ forward_speed        │ 0.20       │ 0-∞     │ 前进速度m/s  │
│ angular_gain         │ -1.20      │ -∞~∞    │ 转向增益    │
│ max_angular_speed    │ 0.60       │ 0-∞     │ 最大角速度  │
│ confidence_threshold │ 0.50       │ 0-1.0   │ 置信度下限  │
└──────────────────────┴────────────┴─────────┴──────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 安全参数 (rk_unitree_driver/cmd_vel_bridge_node)           │
├──────────────────────┬────────────┬─────────┬──────────────┤
│ 参数名               │ 默认值     │ 范围    │ 说明         │
├──────────────────────┼────────────┼─────────┼──────────────┤
│ max_linear_x         │ 0.20       │ 0-1.0   │ 最大线速度  │
│ max_angular_z        │ 0.60       │ 0-2.0   │ 最大角速度  │
│ cmd_timeout_sec      │ 0.50       │ 0.01-5  │ 命令超时    │
│ stop_publish_count   │ 3          │ 1-10    │ 停止重复次  │
│ stop_publish_period_ │ 0.05       │ 0-1     │ 停止间隔秒  │
│ sec                  │            │         │             │
└──────────────────────┴────────────┴─────────┴──────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 感知参数 (硬编码)                                          │
├──────────────────────┬────────────┬─────────┬──────────────┤
│ 参数名               │ 值         │ 范围    │ 说明         │
├──────────────────────┼────────────┼─────────┼──────────────┤
│ LineTracker Rate     │ 0.5s       │ fixed   │ 发布周期    │
│ SignDetector Rate    │ 1.0s       │ fixed   │ 发布周期    │
│ ItemTag Rate         │ 1.0s       │ fixed   │ 发布周期    │
│ LineTrack.confidence │ 0.95       │ fixed   │ 感知置信度  │
│ LineTrack.lateral_   │ [-0.04,    │ pattern │ 模拟偏差    │
│ error pattern        │  0.04]     │ cycling │ 循环模式    │
└──────────────────────┴────────────┴─────────┴──────────────┘
```

---

## 7. 系统总体性能指标 (System Performance)

```
延迟分析 (Latency Analysis):

感知发布 → 控制处理 → 命令发送 → 硬件执行
    ↓           ↓           ↓          ↓
   0ms        0-10ms      10-30ms    30-100ms
            (线性变换)   (API转换)   (网络/HW)
   
   总端到端延迟: ~100-140ms (单跳)

消息频率分析 (Message Rate):

话题                  发布频率    消息大小    带宽
─────────────────────────────────────────────────
/perception/line_track    2Hz      ~200B     0.4 KB/s
/perception/sign_det.     1Hz      ~400B     0.4 KB/s
/perception/item_tags     1Hz      ~600B     0.6 KB/s
/navigation/cmd_vel       ~2Hz     ~100B     0.2 KB/s
/api/sport/request        ~2Hz     ~300B     0.6 KB/s

总系统带宽: ~2.2 KB/s (非常轻)

CPU占用估计 (CPU Usage Estimation):

节点                        单线程时间    频率    CPU%
─────────────────────────────────────────────────────
mock_line_tracker_node       <1ms      2Hz    <0.2%
mock_sign_detector_node      <1ms      1Hz    <0.1%
mock_item_tag_node           <1ms      1Hz    <0.1%
line_follower_node           <2ms      2Hz    <0.4%
cmd_vel_bridge_node          <2ms      2Hz    <0.4%
mission_state_machine        <10ms     1Hz    <0.1%
(其他服务器节点通常待机)

总估计: <1.5% (Mock系统)
       5-15% (Real系统 - 受限于硬件通信)
```

---

## 8. 实际场景执行时间轴 (Execution Timeline Example)

```
t=0s:
  └─ ros2 launch rk_bringup mock_competition.launch.py auto_start:=true
     └─ Launch系统启动8个节点
     └─ 定时器初始化
     └─ 所有节点进入 spin() 循环

t=0-1s:
  ├─ mock_line_tracker: 发布 LineTrack (#1) @ 0.5s
  ├─ line_follower: 订阅并处理，发布 Twist @ 0.5s
  ├─ cmd_vel_bridge: 订阅和验证，发送 API @ 0.5s
  ├─ mock_sign_detector: 发布 SignDetection @ 1.0s
  └─ mock_item_tag_node: 发布 ItemTag @ 1.0s

t=1s:
  └─ mission_state_machine auto_start timer fires
     ├─ 创建 RunMission.Goal(start=true)
     └─ 发送到自身的 /mission/run 动作服务器

t=1-2s:
  ├─ mission_state_machine execute_callback 启动
  ├─ Stage 0: PRECHECK (完成)
  └─ Stage 1: START
     ├─ 创建 ExecuteMotion.Goal(motion_name="start")
     └─ 等待 mock_locomotion_server 完成 (~1s)

t=2-3s:
  └─ mock_locomotion_server 完成，返回结果
     ├─ mission 继续 Stage 2: LINE_FOLLOW
     ├─ 创建 ExecuteMotion.Goal(motion_name="line_follow")
     └─ 等待完成 (~1s)

t=2-3s (并行):
  ├─ 感知和导航循环继续运行
  ├─ LineTracker 发布数据
  ├─ LineFollower 处理数据
  └─ CmdVelBridge 转发命令

t=3-13s:
  └─ 继续阶段2-11的执行
     ├─ 每个阶段 ~1s
     ├─ ARM阶段调用 MockArmServer
     └─ 其他阶段调用 MockLocomotionServer

t=13s:
  └─ Stage 12: DONE
     ├─ goal_handle.succeed()
     └─ 返回 RunMission.Result(success=True)

t>13s:
  ├─ 任务完成
  ├─ 节点继续运行，等待下一个任务请求
  └─ 感知和导航系统继续循环
```

---

**生成时间**: 2026-05-09  
**用于**: RK Inspection 系统分析
**版本**: 1.0
