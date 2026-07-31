# B1 Maze Perception Dry Run

## 1. 安全边界

`maze_perception_dry_run.py` 只订阅点云和 odometry，并发布诊断字符串：

- 输入：`sensor_msgs/msg/PointCloud2`。
- 输入：`nav_msgs/msg/Odometry`。
- 输出：`std_msgs/msg/String`，默认 topic 为
  `/maze/perception/dry_run_status`。
- 不发布 `Twist`。
- 不调用 `/api/sport/request`。
- 不导入或调用 Unitree 运动 SDK。
- `FORWARD/TURN_LEFT/TURN_RIGHT/STOP` 只是诊断建议，不是运动命令。

真机运行时不要启动自动运动桥、gait Action Server 或 command mux。

## 2. 真机环境

Go2 当前需要 Foxy 基础环境和配套构建的 CycloneDDS `0.10.2`。每个新终端
执行：

```bash
source /opt/ros/foxy/local_setup.bash

export AMENT_PREFIX_PATH="/home/unitree/cyclonedds_ws/install/rmw_cyclonedds_cpp:/home/unitree/cyclonedds_ws/install/cyclonedds:${AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="/home/unitree/cyclonedds_ws/install/rmw_cyclonedds_cpp/lib:/home/unitree/cyclonedds_ws/install/cyclonedds/lib:${LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
```

检查配套库：

```bash
ros2 pkg prefix rmw_cyclonedds_cpp

ldd /home/unitree/cyclonedds_ws/install/rmw_cyclonedds_cpp/lib/librmw_cyclonedds_cpp.so \
  | grep libddsc
```

两项都必须指向 `/home/unitree/cyclonedds_ws/install`，不能混用
`/opt/ros/foxy` 的 CycloneDDS `0.7.0`。

## 3. 真机干跑

机器人保持静止：

```bash
cd ~/big-dog-league

python3 scripts/maze_perception_dry_run.py \
  --ros-args \
  --params-file config/maze_perception_dry_run.yaml
```

真机配置使用 `front_angle=20deg` 和 `print_rate=10Hz`。启动后可通过
`ros2 param get /maze_perception_dry_run front_angle` 与 `print_rate` 核对；
不要在当前Foxy环境中同时加载参数文件并依赖同名 `-p` 覆盖。

查看结构化状态：

```bash
ros2 topic echo /maze/perception/dry_run_status
```

日志和 JSON 包含：

- `state`：`CLEAR`、`BLOCKED` 或 `STALE`。
- `advice`：`FORWARD`、`TURN_LEFT`、`TURN_RIGHT` 或 `STOP`。
- 两个输入的消息 age。
- raw yaw 和跨正负 pi 连续的累计 `turn`。
- 五扇区距离、点数、有限点数以及总有效点数。
- 障碍和清除持续帧计数。
- `dry_run: true` 安全标识。

## 4. 决策规则

任一输入缺失、非法或超时会立即进入：

```text
state=STALE advice=STOP
```

点云字段缺失、点数不足或有限点数不足同样视为非法输入，不能因为 topic 仍在
发布就恢复为 `CLEAR`。

点云恢复后，必须连续达到 `clear_confirm_frames` 才会进入 `CLEAR`。检测到
近障碍时，必须连续达到 `blocked_confirm_frames` 才会进入 `BLOCKED`；确认
期间建议已经变为 `STOP`，不会继续建议前进。

`front_block_enter/exit` 控制正前方，`diagonal_block_enter/exit` 控制左右前方。
exit 大于 enter，障碍距离处于两者之间时保持原状态，避免边界抖动。

`front`、`left_front`、`right_front` 输出点云坐标原点到障碍的径向距离。
当前 `/utlidar/cloud_base` 的 `frame_id` 为 `base_link`，因此这些值不能直接
等同于从物理雷达外壳起量的卷尺距离。
`left`、`right` 输出墙面到 `base_link` 中线的横向净距 `|y|`。真机 45cm
挡板在正侧方没有稳定回波，因此侧距在正侧点不足时接受
`side_projection_angle_min..max` 范围内、位于
`side_projection_x_min..max` 前向窗口中的斜视墙点。一个斜前点可同时支持
斜前障碍距离和侧墙净距，但 `valid_points` 只计一次。

`front_angle` 只定义正前总角宽，`diagonal_angle_max` 和
`side_angle_max` 独立定义斜前、正侧边界。真实正侧点达到
`side_min_points` 时优先使用；否则斜前候选点必须沿 x 方向达到
`side_projection_min_x_span`，并在横向
`side_projection_lateral_tolerance` 内形成墙面簇，才可投影为侧墙。该检查用于
拒绝近似固定 x 的前挡板边缘，避免它在拐角前被误报为左右两侧同时只有约14cm。

拐角处真实侧墙的可见纵向长度可能暂时小于
`side_projection_min_x_span`。此类横向成簇的点只作为短墙连续性候选：必须先有
强确认侧墙，候选与缓存距离之差不超过 `side_continuity_tolerance` 才能延续，
并且延续结果只能保持或减小原净距，不能扩大安全空间。短墙候选不能在启动、
点云断流或缓存清空后自行生成侧距，来源标记为 `continued_projected`。

低矮挡板可能在相邻点云帧中交替缺少左、右侧回波。B1 使用
`side_hold_frames` 对每侧最近有效距离做短时保留，并通过
`sector_sources=held_direct/held_projected` 和 `sector_hold_frames` 明确标记。
近墙回波消失时，远处挡板仍可能产生有限距离而不是 `n/a`；因此超过
`side_rise_tolerance` 的突然变远也要连续确认，保留期间来源标记为
`held_rise_direct/held_rise_projected`。更近的有效量测会立即覆盖缓存；连续缺测
或变远超过上限、点云断流或解析失败都会清空缓存，因此该机制不能跨传感器失效
继续提供旧墙距。

进入 `BLOCKED` 后比较：

```text
left_score  = min(left_front, left)
right_score = min(right_front, right)
```

选择空间较大的一侧。两侧都小于 `turn_min_clearance` 时建议 `STOP`。已选转向
会保持，直到另一侧至少多出 `turn_switch_margin`，避免左右反复切换。

点云区域没有返回时距离为 `null/n/a`。B1 的抽象转向建议仍将单个无回波区域
按最大量程处理；B2 不采用这一乐观假设，在走廊内缺少必需侧距时等待或进入
`FAULT_STOP`。消息失效与单个区域无返回仍是两个不同状态。

## 5. 参数标定

配置文件中的距离参数只是初始值，不能直接作为比赛 PASS 参数：

| 参数 | 作用 |
|---|---|
| `front_angle` | 正前扇区总角宽；窄通道过大会混入两侧挡板 |
| `diagonal_angle_max` | 斜前扇区相对正前的最大绝对角度 |
| `side_angle_max` | 正侧扇区最大绝对角度，之后视为后方 |
| `front_block_enter/exit` | 正前方障碍进入/退出阈值 |
| `diagonal_block_enter/exit` | 左前、右前障碍进入/退出阈值 |
| `blocked_confirm_frames` | 障碍成立所需连续点云帧 |
| `clear_confirm_frames` | 清除成立所需连续点云帧 |
| `turn_min_clearance` | 允许建议转向的最低侧向空间 |
| `turn_switch_margin` | 改变已选转向所需的空间优势 |
| `preferred_turn` | 左右空间相近时的默认方向 |
| `cloud_stale_timeout` | 点云超时后进入 `STALE` |
| `odom_stale_timeout` | odometry 超时后进入 `STALE` |
| `min_cloud_points` | 每帧允许的最低原始点数 |
| `min_finite_points` | 每帧允许的最低非 NaN/Inf 点数 |
| `side_projection_angle_min/max` | 可投影为侧墙的斜前角度窗口 |
| `side_projection_x_min/max` | 侧墙投影使用的前向 x 窗口 |
| `side_projection_min_x_span` | 投影点必须覆盖的最小纵向 x 跨度 |
| `side_projection_lateral_tolerance` | 同一投影墙面簇允许的横向厚度 |
| `side_min_points` | 输出单侧墙距离所需的最少点数 |
| `side_hold_frames` | 单侧稀疏缺测时保留最近有效值的最大帧数 |
| `side_rise_tolerance` | 无需持续帧确认即可接受的单帧侧距增大量 |
| `side_continuity_tolerance` | 短墙候选延续已确认侧距时允许的横向差值 |

最终阈值必须使用 B0 五方向纸箱测试和实际停止距离确定。

2026-07-30 真机静止采样中，30 帧有效高度点的水平角只覆盖约
`-45..+60 deg`，`|angle| >= 60 deg` 的点数为 0；左右墙斜前回波的横向
距离约为右侧 `0.22..0.25m`、左侧 `0.24..0.26m`。以上仅证明投影方法有
可用输入，不代表动态迷宫测试通过。

2026-07-31 在57cm入口通道完成了 `front_angle=20deg` 两点静态验证：

- 第一位置从物理雷达外壳到前挡板约 `0.93m`，B1在 `base_link` 下稳定输出
  `1.196..1.207m`，状态保持 `CLEAR`。
- 使用固定参考点向前移动，卷尺距离约由 `0.93m` 变为 `0.80m`；51个B1
  样本的 `front` 为最小 `1.025m`、中位 `1.037m`、最大 `1.051m`。
- B1中位距离变化约 `0.162m`，与人工约 `0.13m` 的变化相差约 `0.032m`；
  左右侧距最小值分别为 `0.242m` 和 `0.219m`，两路新鲜度均通过。
- 同一位置使用45度配置时，`front` 会在约 `0.65..1.20m` 间跳变并持续
  `BLOCKED`。因此B1真机配置改用20度，并增加窄通道侧墙回归测试。

2026-07-31 首次左转人工干跑在 `CORNER_APPROACH` 进入
`FAULT_STOP/corner_side_clearance_unsafe`。故障后样本中 `front` 约
`0.47m`，左右侧距同时降到约 `0.14m`，但居中误差接近零。代码审计确认旧实现
会把约 `x=0.47m` 的前挡板边缘投影为左右侧墙，且20度 `front_angle` 会连带
把直接侧扇区压缩到30至50度。现已解耦扇区边界、优先直接侧墙并增加投影 x
跨度检查。该修复必须通过本次 rosbag 回放和原位静态复测后，才能重新开始
人工行走，当前不记录为真机 PASS。

以上只确认静止距离的稳定性和变化方向。`base_link` 到物理雷达的精确安装
偏移、动态接近拐角、五次转向和全程零碰撞仍需继续真机验证。

## 6. 模拟测试

终端 1 启动干跑节点，并把真实输入 remap 到模拟 topic：

```bash
python3 scripts/maze_perception_dry_run.py \
  --ros-args \
  --params-file config/maze_perception_dry_run.yaml \
  -r /utlidar/cloud_base:=/maze_sim/cloud \
  -r /utlidar/robot_odom:=/maze_sim/odom
```

终端 2：

```bash
python3 scripts/maze_perception_simulator.py \
  --ros-args \
  --params-file config/maze_perception_dry_run.yaml
```

模拟器只发布 PointCloud2 和 Odometry，循环场景：

```text
clear
blocked_choose_left
hysteresis_band
clear_after_block
blocked_choose_right
boxed_stop
cloud_stale
odom_stale
```

预期建议依次覆盖 `FORWARD`、`TURN_LEFT`、滞回保持、`TURN_RIGHT`、`STOP`，
并在两个 stale 场景立即输出 `STALE/STOP`。模拟结果不能填写为真机 PASS。

## 7. Rosbag 测试

记录输入：

```bash
ros2 bag record \
  -o b1_maze_inputs \
  /utlidar/cloud_base \
  /utlidar/robot_odom
```

离线回放时先启动干跑节点，再播放：

```bash
ros2 bag play b1_maze_inputs
```

暂停 rosbag 超过 stale timeout 时应立即得到 `STALE/STOP`；继续播放后必须重新
完成持续帧确认才能恢复 `CLEAR` 或 `BLOCKED`。

## 8. 真机进入下一阶段的条件

- 五方向纸箱距离和检测连续性通过预设阈值。
- 10 分钟内无意外 `STALE`。
- yaw 左右符号、90 度误差和正负 pi 跨界完成实测。
- `CLEAR/BLOCKED` 不在阈值附近反复抖动。
- 近障碍时建议停止或转向，封闭时只建议 `STOP`。
- 真实 Move/Stop/Stand、急停和停止延迟仍未接入，B1 不允许控制机器人。
