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

`front`、`left_front`、`right_front` 输出雷达原点到障碍的径向距离。
`left`、`right` 输出墙面到 `base_link` 中线的横向净距 `|y|`。真机 45cm
挡板在正侧方没有稳定回波，因此侧距同时接受
`side_projection_angle_min..max` 范围内、位于
`side_projection_x_min..max` 前向窗口中的斜视墙点。一个斜前点可同时支持
斜前障碍距离和侧墙净距，但 `valid_points` 只计一次。

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
| `side_min_points` | 输出单侧墙距离所需的最少点数 |

最终阈值必须使用 B0 五方向纸箱测试和实际停止距离确定。

2026-07-30 真机静止采样中，30 帧有效高度点的水平角只覆盖约
`-45..+60 deg`，`|angle| >= 60 deg` 的点数为 0；左右墙斜前回波的横向
距离约为右侧 `0.22..0.25m`、左侧 `0.24..0.26m`。以上仅证明投影方法有
可用输入，不代表动态迷宫测试通过。

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
