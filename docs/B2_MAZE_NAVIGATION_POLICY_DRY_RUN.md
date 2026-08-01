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
| `CORNER_APPROACH` | 跟随转向对侧墙减速接近，并确认开放侧 |
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
完整传感器 STALE 仍立即锁止。转弯中的单帧净空不足立即将诊断速度清零，连续
达到 `side_unsafe_confirm_frames` 才锁止。

B1 会按固定频率重发 freshness 快照。B2 只允许递增的 `cloud_sequence` 推进
持续帧计数；相同序号只更新显示数据，不能用于确认转向开口或危险帧。

## 4. 转向安全包络

开始移动转向前至少要求：

- 正前距离不小于机身半长加安全余量，约 `0.38m`。
- 目标开放侧不小于 `turn_open_distance_m`，默认 `0.50m`。
- 目标斜前距离不小于约 `0.38m`。
- 对侧距离不小于机身半宽加安全余量，约 `0.185m`。

目标侧 `null/n/a` 只有在拐角状态中才可解释为开放侧。对侧、正前或目标斜前
为 `null/n/a` 时，`moving_turn_sweep_safe=false`，不得开始转向。

严格开口条件连续确认后才锁存 `TURN_LEFT/RIGHT`。转向过程中，目标侧可能重新
看到身旁墙端，因此不再要求它持续大于 `0.50m`；此时仍要求正前和目标斜前不小
于约 `0.38m`、对侧及任何重新出现的目标侧回波不小于 `0.185m`。这一区分只
避免开口远近回波切换造成误锁止，不降低机身即时碰撞余量。

接近已知左转时只用右墙计算居中误差；接近右转时只用左墙。开口侧的远墙回波
不会再把诊断角速度拉向开口。终端 D 通过 `center_reference` 显示当前参考墙，
并分别显示当前转向安全结果和严格启动条件。

对侧墙参考从 `front <= corner_approach_distance_m` 的第一帧立即生效，不等待
`corner_confirm_frames` 完成。持续帧只决定状态名称何时切换，不能让确认期间的
开口侧远墙生成相反方向的居中建议。

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

B2 的显示和 Yaw 微调至少需要约 `10Hz` 的 B1 JSON 快照，因此真机配置已将
`print_rate` 固定为10Hz；持续帧只按其中不同的 `cloud_sequence` 计数。
当前Foxy环境不应在加载参数文件后再用同名
`-p` 猜测覆盖结果；启动后应使用 `ros2 param get` 核对最终值。

B2 真机配置同样以 `10Hz` 输出终端 D 所需状态。人工接近和尺量允许最多
`1800s`；转向及重捕获为 `120s`。这些值只给操控员分段松杆观察留出时间，
不覆盖点云/Odom STALE、侧距不足和扫掠空间保护。未来接入自主运动输出前，
必须根据实测速度重新缩短超时。

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

### 人工连续标定录包

不运行 B2/D、只采集原始点云、Odom、B1 和人工检查点时，使用仓库内的自助脚本：

```bash
cd ~/big-dog-league

bash scripts/maze_manual_recorder.sh start \
  --iface eth0 \
  --delay 5 \
  --marker "MAZE_START front_cm=107 left_front_cm=19 left_rear_cm=19 right_front_cm=21 right_rear_cm=21 parallel=yes"
```

脚本先启动只读 B1，倒计时结束后才启动 rosbag，并在 Topic 发现完成后写入
`/maze/operator_marker`。检测到 SDK UDP 服务端、速度转发器、其他 B1 或其他
rosbag 时会拒绝启动。该脚本不启动 B2、D、Twist 发布器或 Unitree API。

查看和安全停止：

```bash
bash scripts/maze_manual_recorder.sh status
bash scripts/maze_manual_recorder.sh stop
```

必须使用 `stop` 让 rosbag 写完 `metadata.yaml`，不能直接关闭 tmux 或删除正在
录制的目录。

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
| `turn_start_confirm_frames` | 转向开口及矩形扫掠包络的连续确认帧数 |
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
- 同一 `cloud_sequence` 重发不能推进转向开口确认。
- 目标开口侧可缺测，但对侧缺测绝不允许开始转弯。
- 接近开口时只跟随转向对侧墙，开口侧跳变不改变居中诊断。
- 转向启动后目标侧墙端回波仍高于 `0.185m` 时不得误报封路。
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

2026-08-01 Round 4 的三次前进点动分别约为 `10cm`、`15cm` 和 `18cm`。
人工干跑配置因此按 `18cm` 上界生成终端D提示，操控员仍须每次点动后立即松杆
并重新测量。接近阶段的尺量和日志核对可能超过数分钟，故接近超时设为
`1800s`；转向和重捕获仍限制为 `120s`，避免静止 Odom 漂移长期累积。
B2仍不发布任何运动命令，接入自主运动前必须重新标定这些超时值。

同轮测试还发现，单帧左侧开口曾使 B2 进入 `TURN_LEFT`，随后左侧稳定距离仅
约 `0.36..0.45m`，低于 `0.50m` 转向开口门限，而旧版终端D仍显示允许左转。
修复后，进入转向前必须连续满足 `turn_start_confirm_frames`；转向或精调期间包络
不足时首帧立即清零诊断值、连续第二帧锁止，终端D也独立覆盖为停止提示。

Round 4 最终停在人工测得前方余量约 `35cm` 的位置。此时 B1 前距约
`0.56..0.59m`、左侧约 `0.36..0.45m`、右侧约 `0.25m`。修复后的 B2 在机械狗
静止时连续12秒保持 `CORNER_APPROACH / waiting_for_turn_opening`，诊断
`desired_vx=0`、`desired_wz=0`，没有再次误入转向。整轮录包位于
`/home/unitree/maze_bags/b2_round4_20260731_235948`，大小约 `1.1GB` 且
`metadata.yaml` 已生成。该轮没有执行实际左转，首弯验收结果为 **未通过/需改进**；
禁止将修复后的静止保护结果记录成整段迷宫或动态转弯 PASS。

2026-08-01 Round 6 在人工前距约 `52cm`、左侧机身间隙约 `22cm`、右侧约
`17cm` 时，B1 左距在 `0.253..0.773m` 间切换，右距稳定在
`0.220..0.265m`。包中同一真实点云曾被 B1 周期快照重复约 `0.42s`；该帧为
`front=0.539m`、`left=0.556m`，旧 B2 将三个相同快照误算成三帧后进入
`TURN_LEFT`。随后左侧回落到 `0.472m`，旧策略连续两帧判定完整扫掠不足并锁止
`turn_sweep_unsafe`，操控员尚未实际转弯。

修复后 B1 输出仅随新 PointCloud2 递增的 `cloud_sequence`，B2 对重复序号不再
推进状态；转向启动后采用即时碰撞余量，而严格 `0.50m` 开口只用于启动确认。
使用 Round 6 原始点云消息时间对 B1 JSON 去重离线重放后，状态仅经过
`WAIT_SENSOR -> CORRIDOR_FOLLOW -> CORNER_APPROACH`，没有误入转向或故障。
录包为 `/home/unitree/maze_bags/b2_round6_20260801_161516`，约 `51MB`。
该结果仍是离线修复验证，Round 6 动态首弯判定为 **未通过/需重新静态复测**。

2026-08-01 Round 8 在入口完成60秒静止门禁：点云约 `14.70Hz`、最大间隔
`0.090s`，Odom约 `149.45Hz`、最大间隔 `0.022s`，600条B1状态均为
`CLEAR`。随后左侧低墙回波连续两帧缺测，B2按配置锁止
`corridor_side_distance_missing`；原始点云回放确认近墙短簇仍存在，并据此完成
B1侧距窗口的离线标定，尚待真机原位复测。

同轮机器狗四角尺量显示机身仍与通道平行，但 B1 的启动累计转角在约
`1369s` 内漂移到约 `66deg`，约为 `3deg/min`。第二次约 `2cm` 直行期间的实际
Odom增量仅约 `0.46deg`，因此不能把长期累计值解释成机身相对通道角度。
B2转弯目标原本就以进入 `TURN_LEFT/RIGHT` 时的当前值为零点；现在 JSON 和
终端D额外显示 `turn_progress_deg`，长期值明确标注为含静止漂移的
`Odom启动累计`。实际90度转向的短时误差仍需真机角度标记验证。

2026-08-01 Round 10 入口人工四角间隙均约 `19.5cm` 时，前150秒共1500帧
B1数据的左、右侧距中位数分别为 `0.252m`、`0.246m`，因此真机配置将
`side_target_m` 从几何假设 `0.285m` 标定为 `0.25m`。一次约 `13cm` 前进后，
人工四角间隙为左前/左后 `17/18cm`、右前/右后 `23/22cm`，确认机身约向左
偏移 `2.5cm` 且机头仅轻微向左。同期右侧B1稳定在约 `0.268m`，左侧却因首个
左开口在 `0.261..0.797m` 间跳变；旧配置仍处于双墙居中并给出错误的向左建议。

修复后 `corner_approach_distance_m=0.98m`，进入该距离后的第一帧即使用右墙
生成首个左弯的居中建议，正式状态仍需三帧确认。按本轮标定值，右距
`0.268m` 会得到负的居中误差，即建议向右恢复，与人工四角测量方向一致。
录包为 `/home/unitree/maze_bags/b2_round10_20260801_213930`；该轮在发现建议方向
冲突后停止，没有继续移动或执行转弯，因此不得记录为动态首弯PASS。

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
