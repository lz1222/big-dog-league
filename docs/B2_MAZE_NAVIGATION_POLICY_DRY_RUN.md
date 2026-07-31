# B2 Maze Navigation Policy Dry Run

## 1. 任务边界

B2 消费 B1 发布的五扇区、Yaw、累计转角和 freshness，生成迷宫状态与候选
`desired_vx/desired_wz`。两个数值仅写入 JSON 诊断：

- 不导入或发布 `geometry_msgs/msg/Twist`。
- 不调用 `/api/sport/request`。
- 不导入或调用 Unitree SDK。
- 不修改巡线、`gait_control_node`、command mux 或运动桥。
- 不根据模拟输出填写真机 PASS。

输入：

```text
/maze/perception/dry_run_status
std_msgs/msg/String
```

输出：

```text
/maze/navigation/dry_run_status
std_msgs/msg/String
```

## 2. 已知迷宫

| 项目 | 数值 |
|---|---:|
| 迷宫外轮廓 | 1.80m × 1.40m |
| 底边分段 | 1.20m 挡板 + 0.60m 入口 |
| 左侧纵向分段 | 0.80m 挡板 + 0.60m 开口 |
| 中部两块纵向挡板 | 各 0.80m |
| 右侧外挡板 | 1.40m |
| Go2 长宽高 | 0.70m × 0.31m × 0.40m |
| 挡板高度 | 0.45m |
| 最窄实际净宽 | 0.57m |
| 固定大方向 | LEFT、LEFT、RIGHT、RIGHT、LEFT |
| 单次名义转角 | 90° |
| 比赛原则 | 无硬限时，零碰板优先 |

以上尺寸按 2026-07-31 修正后的现场平面图记录。`1.40m` 是迷宫整体
纵深，不是通道宽度；`corridor_width_m` 必须继续使用包含接头和底座影响后的
实测最窄净宽 `0.57m`。本次修正没有改变通道拓扑，因此五次大方向保持不变。

当前 B2 不使用固定直行距离猜测拐角，只根据实时前方距离和墙面开口进入转弯。
在起点 `base_link` 位置、挡板厚度和各开口端点完成复测前，不从上述外轮廓
尺寸推导开环行驶时间或里程计目标。

通道居中时，机身两侧几何余量各约：

```text
(0.57 - 0.31) / 2 = 0.13m
```

机身矩形的半对角线约 `0.383m`。加入 `0.03m` 安全余量后，纯原地旋转所需
扫掠直径约：

```text
2 × (0.383 + 0.03) = 0.826m
```

该值大于 `0.57m`，因此策略明确输出：

```text
in_place_rotation_fits_corridor=false
```

B2 不会把纯原地转向视为可行方案，而是检查开放侧、斜前、正前和对侧余量，
给出移动转向诊断。

## 3. 状态机

| 状态 | 含义 |
|---|---|
| `WAIT_SENSOR` | 等待 B1 连续新鲜 |
| `CORRIDOR_FOLLOW` | 根据左右墙距离计算居中修正 |
| `CORNER_APPROACH` | 减速接近转弯起点并确认开放侧 |
| `TURN_LEFT` | 使用连续 `turn_rad` 左转约90° |
| `TURN_RIGHT` | 使用连续 `turn_rad` 右转约90° |
| `TURN_FINE_ALIGN` | 接近目标角后降低角速度并连续确认 |
| `CORRIDOR_REACQUIRE` | 转后重新确认前方、中心误差和航向 |
| `REVERSE_RECOVERY` | 前方过近时的反向诊断候选 |
| `FINISHED` | 五次转向完成且雷达连续确认出口开放 |
| `FAULT_STOP` | 传感器失效、余量不足或状态超时 |

`FAULT_STOP` 和 `FINISHED` 均锁定为零诊断速度，需重启节点才能开始新一轮。

`WAIT_SENSOR` 必须连续收到左右侧墙净距后才可进入
`CORRIDOR_FOLLOW`。进入走廊后，非预期侧距缺失会先按
`side_missing_confirm_frames` 输出零诊断速度并要求保持静止，连续缺失才锁定
`FAULT_STOP`；接近已知拐角时，仅允许预期转向侧以无墙回波表示开口，对侧墙、
前方和目标斜前仍须有实测距离。侧向空间不足的首帧同样立即输出零诊断速度，
连续达到 `side_unsafe_confirm_frames` 才锁止；该确认不降低 `0.185m` 门限。
完整传感器 STALE 以及转弯中的瞬时扫掠包络不足仍立即锁止。

## 4. 转向安全包络

默认移动转向至少要求：

- 正前距离不小于机身半长加安全余量，约 `0.38m`。
- 目标开放侧不小于 `turn_open_distance_m`，默认 `0.50m`。
- 目标斜前距离不小于约 `0.38m`。
- 对侧距离不小于机身半宽加安全余量，约 `0.185m`。

目标侧 `null/n/a` 只有在拐角状态中才可解释为开放侧。对侧、正前或目标斜前
为 `null/n/a` 时，`moving_turn_sweep_safe=false`，不得开始转向。

这些阈值基于 `base_link` 位于机身几何中心的假设。真机如果不满足该假设，
必须先测量雷达原点到机身前后左右边缘的偏移，再修改参数。

B1 没有后扇区，因此 `REVERSE_RECOVERY` 输出中固定包含：

```text
reverse_rear_visibility_confirmed=false
```

反向速度目前只用于离线策略检查。B3 未增加后向保护或验证“沿刚走过的短路径
回退”之前，不得直接执行该值。

## 5. 真机只读运行

先使用 B1 已验证的 Foxy 和 CycloneDDS 环境。终端1：

```bash
cd ~/big-dog-league

python3 scripts/maze_perception_dry_run.py \
  --ros-args \
  --params-file config/maze_perception_dry_run.yaml
```

B2 的持续帧和 Yaw 微调至少需要约 `10Hz` 的 B1 JSON 快照，因此真机配置
已将 `print_rate` 固定为10Hz。当前Foxy环境不应在加载参数文件后再用同名
`-p` 猜测覆盖结果；启动后应使用 `ros2 param get` 核对最终值。

B2 真机配置同样以 `10Hz` 输出终端 D 所需状态，并将人工操作相关的
`corner_timeout_sec`、`turn_timeout_sec`、`reacquire_timeout_sec` 设为30秒。
延长值只给操控员分段松杆观察留出时间，不覆盖点云/Odom STALE、侧距不足和
扫掠空间保护。未来接入自主运动输出前，必须根据实测速度重新缩短超时。

2026-07-31 入口静态标定后，B1配置已将 `front_angle` 从45度改为20度；
45度会在57cm窄通道中把侧墙回波混入 `front`。B2的前方阈值使用
`base_link` 下的B1距离，不直接使用从物理雷达外壳量得的卷尺距离。

终端2：

```bash
cd ~/big-dog-league

python3 scripts/maze_navigation_dry_run.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml
```

终端 D 启动中文实时操作提示器：

```bash
cd ~/big-dog-league

python3 scripts/maze_operator_monitor.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml
```

该提示器只订阅 `/maze/navigation/dry_run_status`，不创建速度发布器，也不调用
Unitree SDK。交互终端会动态显示中文操作指令、五扇区距离、路线进度、转角误差、
居中误差和数据新鲜度。B2 状态超过 `status_stale_timeout_sec` 未更新时，终端 D
会直接显示“立即停止”。

人工干跑时按终端 D 的当前指令操作：

| 中文指令 | 操作 |
|---|---|
| 保持静止 | 不推动遥控杆，等待传感器确认 |
| 短促前进 | 每次前进约2至5cm，松杆后重新观察 |
| 更小步前进 | 每次前进约1至3cm，等待明确转向状态 |
| 停止并等待 | 不前进、不提前转向，检查开口和距离 |
| 开始左转/右转 | 少量前进加对应转向，禁止纯原地旋转 |
| 松杆保持，等待角度确认 | 当前误差已进入4度范围，不再增加转角 |
| 缓慢进入新通道 | 短促进入并居中，等待下一段走廊状态 |
| 立即停止 | 松开遥控器；故障锁止需排障后重启 B2 |

`REVERSE_RECOVERY` 即使包含负的 `desired_vx`，终端 D 也只会提示停止。原因是
B1 没有后向扇区，操控员不得把该诊断候选直接执行为倒退动作。

需要保存未经整理的原始 JSON 时，可另开终端执行：

```bash
ros2 topic echo /maze/navigation/dry_run_status
```

此模式不会让静止机器人自动推进全部转向。它只用于确认真实 B1 输入是否能让
B2 正确进入 `WAIT_SENSOR`、`CORRIDOR_FOLLOW` 或保护状态。

如果静止放置时策略已经输出非零诊断速度，Yaw 不会随之变化，因此
`corner_approach_timeout` 或 `turn_timeout` 是预期的干跑保护结果，不代表
机械狗执行了转向。

## 6. 模拟完整路线

终端1启动 B2：

```bash
python3 scripts/maze_navigation_dry_run.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml
```

终端2启动名义模拟：

```bash
python3 scripts/maze_navigation_simulator.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml
```

模拟器按以下顺序生成 B1 JSON：

```text
走廊 -> 接近 -> 左转 -> 重获
走廊 -> 接近 -> 左转 -> 重获
走廊 -> 接近 -> 右转 -> 重获
走廊 -> 接近 -> 右转 -> 重获
走廊 -> 接近 -> 左转 -> 重获
出口开放
```

预期最终出现：

```text
state=FINISHED
route=5/5
desired_vx=0
desired_wz=0
```

故障模拟每次都要重启 B2，因为 `FAULT_STOP` 会锁定：

```bash
# B1 STALE
python3 scripts/maze_navigation_simulator.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml \
  -p sim_scenario:=sensor_stale

# Yaw 不变化，触发 turn_timeout
python3 scripts/maze_navigation_simulator.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml \
  -p sim_scenario:=turn_timeout

# 目标侧空间不足，触发反向候选并最终保护停止
python3 scripts/maze_navigation_simulator.py \
  --ros-args \
  --params-file config/maze_navigation_dry_run.yaml \
  -p sim_scenario:=blocked_turn
```

## 7. Rosbag

记录用于重放策略的 B1 输入：

```bash
ros2 bag record \
  -o b2_maze_policy_input \
  /maze/perception/dry_run_status
```

回放策略时只需启动 B2，再播放包含 B1 状态的包：

```bash
ros2 bag play b2_maze_policy_input
```

需要保存 B2 结果时，可在另一个 bag 中记录
`/maze/navigation/dry_run_status`，避免回放时出现两个 B2 输出发布者。

如果 rosbag 只包含 `/utlidar/cloud_base` 和 `/utlidar/robot_odom`，应先启动
B1，再启动 B2，最后播放 rosbag。

## 8. 主要参数

| 参数组 | 作用 |
|---|---|
| `robot_*`、`corridor_width_m` | 机身和迷宫几何 |
| `footprint_safety_margin_m` | 扫掠和碰撞附加余量 |
| `route_directions` | 固定五次大方向 |
| `*_confirm_frames` | 状态持续帧确认 |
| `side_missing_confirm_frames` | 必需侧距连续缺失后锁止的确认帧数；确认中诊断速度为零 |
| `side_unsafe_confirm_frames` | 侧向低净空连续锁止帧数；首帧诊断速度立即归零 |
| `manual_step_distance_cm` | 终端D显示的单次点动保守位移；应使用实测上界 |
| `corridor_vx` 等 | 仅用于 JSON 的候选速度 |
| `center_kp`、`side_target_m` | 走廊居中 |
| `corner_approach_distance_m` | 进入转弯接近状态 |
| `turn_start_distance_m` | 允许开始移动转向的前方距离 |
| `front_emergency_distance_m` | 前方过近保护阈值 |
| `turn_open_distance_m` | 目标侧开放距离 |
| `turn_angle_deg` | 单次目标累计转角 |
| `turn_tolerance_deg` | 转角完成误差 |
| `*_timeout_sec` | 状态超时保护 |

配置中的速度和距离是初始工程值，不是真机标定结果。

## 9. 本地检查

```bash
python3 -m compileall scripts tests

python3 -m unittest discover \
  -s tests \
  -v
```

单元测试覆盖：

- 57cm通道无法容纳矩形机身纯原地旋转。
- 走廊左右偏差的修正方向。
- 开放侧和被阻塞侧的移动转向包络。
- 单帧侧方低余量先停住，连续低余量才锁定 `FAULT_STOP`。
- 走廊左右侧距缺失不会被当作开放空间。
- 单帧侧距缺失先暂停，连续缺失才锁定故障。
- 目标开口侧可缺测，但对侧缺测绝不允许开始转弯。
- 斜前墙点投影为左右横向净距，并受最小点数保护。
- 传感器 STALE 锁定 `FAULT_STOP`。
- Yaw 不变化触发转向超时。
- `LEFT, LEFT, RIGHT, RIGHT, LEFT` 全流程进入 `FINISHED`。

## 10. 仍需真机验证

2026-07-31 首次左转干跑在进入 `CORNER_APPROACH` 后触发
`corner_side_clearance_unsafe`。记录中的左右侧距同时约为 `0.14m`，后续确认
为前挡板边缘污染 B1 侧距，不是操控员偏离中心。B2 的 `0.185m` 侧向安全阈值
未降低。修复后的回放已能进入 `TURN_LEFT`，约完成74度后对侧侧距连续缺失，
随后还出现 `right=0.176m` 的低余量量测；旧轨迹仍应判为未通过。静止联调中，
单帧缺测会正确显示“停止并等待”且恢复后继续，不再立即锁死。下一次人工干跑
必须在首次暂停提示时松杆，并通过更早开始弧线、减小转向量或重新定位获得持续
大于 `0.185m` 的对侧余量。

2026-07-31 Round 3 中，默认步态最轻前进点动实测约 `8cm`。全程解析的
`12144` 帧 B1 日志中仅一帧右侧距为 `0.184m`，前后帧分别约为 `0.223m`
和 `0.226m`，随后静止101帧右侧稳定在 `0.242..0.259m`。因此保留
`0.185m` 门限，并增加两帧确认：首帧必须停住，只有连续第二帧仍低才锁止。
该记录是人工干跑诊断，不代表动态转弯或整场迷宫通过。

2026-08-01 Round 4 的两次前进点动分别约为 `10cm` 和 `15cm`。人工干跑
配置因此按 `15cm` 上界生成终端D提示，操控员仍须每次点动后立即松杆并重新
测量。尺量和日志核对可能超过数分钟，纯诊断节点的拐角、转向和重捕获超时设为
`1800s`；B2仍不发布任何运动命令，接入自主运动前必须重新缩短这些超时值。

- `base_link` 相对机身几何中心的真实偏移。
- 雷达对45cm白色挡板、接头和底座的稳定返回。
- 57cm净宽下实际动态足端宽度。
- 移动转向轨迹能否避开两个拐角的扫掠区域。
- `Move(0,0,0)`、`StopMove`、watchdog 和遥控急停距离。
- 真实最低有效前进速度、转向速度和停止距离。
- 后向保护缺失时是否完全禁用反向。
- 五次转向后雷达出口开放条件的可靠性。

完成以上项目之前，B2 只能标记为“策略与离线测试完成”，不能标记为“真机迷宫
通过”。
