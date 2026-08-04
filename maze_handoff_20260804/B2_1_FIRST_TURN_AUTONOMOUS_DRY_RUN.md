# B2.1 第一弯自主短段规划 Dry Run

## 1. 本轮边界

本模块只设计入口后的第一个左弯。它直接读取：

```text
/utlidar/cloud_base  sensor_msgs/msg/PointCloud2  frame=base_link
/utlidar/robot_odom  nav_msgs/msg/Odometry
```

唯一输出为：

```text
/maze/first_turn/dry_run_status  std_msgs/msg/String(JSON)
```

节点不发布速度，不调用 `/api/sport/request`，不调用 Unitree SDK，也没有把
诊断候选交给运动桥的接口。达到第一弯后固定输出 `TURN_COMPLETE`、
`route=1/5`，不会生成第二弯或完整迷宫路线。

“人工监护下自主短段执行、每段 StopMove 并重新规划”是下一执行阶段的验证
方式。本轮只实现其规划、安全门控和执行跟踪纯逻辑；ROS 节点不会确认短段
执行握手。

## 2. 文件

| 文件 | 用途 |
|---|---|
| `scripts/maze_first_turn_core.py` | 局部图、动态足迹、连续扫掠、候选和状态机纯逻辑 |
| `scripts/maze_first_turn_dry_run.py` | 只读 ROS2 Foxy 节点 |
| `scripts/maze_round15_replay_core.py` | 用历史局部图和后续 Odom 回放旧实际轨迹 |
| `scripts/maze_round15_replay_analyzer.py` | Round15 只读采集与 JSON 证据汇总节点 |
| `config/maze_first_turn_dry_run.yaml` | 第一弯全部参数和未标定运动模型 |
| `config/maze_round15_replay.yaml` | Round15 接触边界与回放几何参数 |
| `tests/test_maze_first_turn_core.py` | 无 ROS、无真机的模拟点云测试 |
| `tests/test_maze_round15_replay_core.py` | 旧实际轨迹回放纯逻辑测试 |

旧 B1/B2、巡线、运动桥、`gait_control_node` 和 command mux 均不参与本节点。

## 3. 局部感知

每个新点云帧在 `base_link` 中重建约 `3m x 3m` 的瞬时二维占据图。处理顺序：

1. 拒绝 NaN、Inf 和字段损坏。
2. 限制局部地图范围。
3. 过滤最小量程和机身区域。
4. 使用高度范围过滤地面及过高点。
5. 将剩余挡板点量化到二维网格。
6. 以确定性 RANSAC/PCA 提取有限墙段、墙端、残差和置信度。

地图不跨帧保留旧障碍，避免机器人移动后把历史挡板留在错误的
`base_link` 位置。

八个角区为：

```text
front, left_front, right_front, left, right,
left_rear, right_rear, rear
```

每个区域独立输出：

- `point_count`：高度和机身过滤后的占据点数。
- `coverage_point_count`：可证明该角度被雷达扫描的有限点数。
- `coverage_bin_count`：有数据的角向桶数量。
- `valid`：当前帧覆盖是否达到门限。
- `age_sec`、`stale`、`usable`：该区域最后有效覆盖的新鲜度。
- `distance_m`：该区占据点的第 10 百分位距离。

后方没有达到点数和角向覆盖门限时，`rear`、`left_rear` 或
`right_rear` 会单独失效，并输出 `rear_coverage_insufficient`；
`REVERSE_SHORT` 必定为 `UNKNOWN` 且不可选择。无点不能解释为空旷。

## 4. 动态矩形足迹

基础足迹使用相对 `base_link` 的四个独立边界：

```text
footprint_front_m
footprint_rear_m
footprint_left_m
footprint_right_m
```

`cloud_uncertainty_margin_m`、`odom_uncertainty_margin_m` 和
`model_uncertainty_margin_m` 加到四边，`gait_sway_margin_m` 加到左右边。
`target_physical_clearance_m` 是扩大足迹之外仍需保留的目标净空。
`stop_tail_margin_m` 不只扩大终点，而是沿候选末端运动方向继续追加一段停止
尾程。

轨迹采样同时受以下三个上限约束：

- 时间间隔 `trajectory_sample_dt_sec`；
- 相邻平移 `trajectory_max_translation_step_m`；
- 相邻转角 `trajectory_max_yaw_step_deg`。

每个采样位姿都检查完整非对称矩形。候选 JSON 输出：

```text
minimum_clearance_m
danger_time_sec
collision_part
danger_point_m
sample_count
```

碰撞证据同时检查有限墙线、墙端和降采样原始障碍点。墙端不确定度会从预测
净空中扣除；名义候选逐点复核全部原始点，扰动候选检查墙模型及所有未建模
障碍点。危险部位只使用 `front_left`、`front_right`、`rear_left`、
`rear_right`、`left_side`、`right_side`、`front_edge`、`rear_edge`。

因此起点和终点都安全、但中间左侧机身碰内板的轨迹仍会被拒绝。

## 5. 第一弯候选

只生成以下六类短段：

1. `FORWARD_SHORT`
2. `OUTSIDE_DIAGONAL_SHORT`，第一左弯的外侧为机器人右侧，因此 `vy < 0`
3. `LEFT_ARC`
4. `LEFT_ARC_OUTSIDE_VY`
5. `REVERSE_SHORT`
6. `FINE_LEFT_ARC`

轨迹生成器明确拒绝 `vx=0, vy=0, wz!=0`，即禁止原地旋转。

候选判定分六级：

| 判定 | 含义 |
|---|---|
| `UNSAFE` | 已预测碰撞或物理净空不足 |
| `UNKNOWN` | 传感器、墙模型、后方覆盖或安全状态不足 |
| `GEOMETRY_SAFE_UNCALIBRATED` | 几何通过，但运动模型或动态余量未标定 |
| `NOMINAL_SAFE` | 名义轨迹通过，但至少一个扰动场景未通过 |
| `ROBUST_SAFE` | 名义轨迹及全部配置扰动均通过 |
| `EXECUTABLE_SAFE` | 仅后续阶段在运动模型和安全链全部验证后允许 |

默认 YAML 中六类运动模型的 `*_calibrated` 和
`margins_calibrated` 全部为 `false`。速度和时长只是离线几何假设，不能用于
真机执行。B2.1-A 永远不输出 `EXECUTABLE_SAFE`，并固定输出：

```json
{"motion_output":false,"execution_allowed":false,"safety_action":"STOP_MOVE_REQUIRED"}
```

## 6. 后退门控

`REVERSE_SHORT` 除普通轨迹安全条件外，还必须同时满足：

- `rear`、`left_rear`、`right_rear` 当前有效且新鲜；
- 后方完整扫掠和停止尾程无碰撞；
- watchdog 状态为 `true` 且新鲜；
- estop 状态为 `false` 且新鲜；
- roll、pitch、线速度和角速度满足稳定门限；
- 后退次数小于 `max_reverse_segments`；
- 累计预测距离不超过 `max_reverse_distance_m`。

纯逻辑状态机只在未来执行层明确确认一个后退脉冲已经完成后才增加计数。
ROS Dry Run 节点没有该确认入口，因此不会自行产生或伪造后退执行记录。

## 7. 状态机

状态范围严格限制为第一弯：

```text
APPROACH_TURN
STOP_AND_SCAN
SELECT_TRAJECTORY
EXECUTE_SHORT_SEGMENT
STOP_AND_RESCAN
REVERSE_RECOVERY
TURN_FINE_ALIGN
CORRIDOR_REACQUIRE
TURN_COMPLETE
FAULT_STOP
```

当前节点可以通过实时或 rosbag 数据进入接近、停止扫描、选择、精调、重捕获
和完成状态。`EXECUTE_SHORT_SEGMENT` 与 `REVERSE_RECOVERY` 只存在于纯逻辑
执行握手中，ROS Dry Run 节点不会触发它们。

以下情况锁止为 `FAULT_STOP`：

- 点云或 Odom 无效、断流或超时；
- Yaw 单帧异常跳变；
- roll/pitch 超限；
- 当前动态足迹已经碰撞或低于硬间隙；
- 阶段超时；
- 当前阶段没有几何安全候选；
- 未来执行跟踪中的位置或 Yaw 偏差超过门限。

初次启动等待两路传感器时保持 `APPROACH_TURN / waiting_initial_sensors`，
不会因为 ROS 节点启动顺序提前锁止。

## 8. 本机运行

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
source install/setup.bash

python3 scripts/maze_first_turn_dry_run.py \
  --ros-args \
  --params-file config/maze_first_turn_dry_run.yaml
```

另一个终端只读查看：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic echo /maze/first_turn/dry_run_status
```

启动前必须确认没有误接运动输出：

```bash
ros2 node info /maze_first_turn_dry_run
```

`Publishers` 中只能看到 `/maze/first_turn/dry_run_status` 和 ROS 日志相关
Topic，不应出现速度 Topic 或 Unitree 请求 Topic。

## 9. Round15 离线重放

停止真实传感器节点和旧 B1/B2 发布者，使用独立 `ROS_DOMAIN_ID`，避免实时数据
与录包数据混合：

终端 A 启动第一弯候选排名：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
source install/setup.bash
export ROS_DOMAIN_ID=31

python3 scripts/maze_first_turn_dry_run.py \
  --ros-args \
  --params-file config/maze_first_turn_dry_run.yaml
```

终端 B 启动旧实际轨迹分析器：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31

python3 scripts/maze_round15_replay_analyzer.py \
  --ros-args \
  --params-file config/maze_round15_replay.yaml
```

终端 C 播放录包：

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
export ROS_DOMAIN_ID=31

ros2 bag info ~/maze_bags/b2_round15_20260802_144706
ros2 bag play ~/maze_bags/b2_round15_20260802_144706
```

终端 D 查看前五候选：

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
export ROS_DOMAIN_ID=31
ros2 topic echo /maze/first_turn/dry_run_status
```

录包播放结束并静默 2 秒后读取：

```bash
cat /tmp/b2_1_a_round15_replay_summary.json
```

重放验收重点：

1. Round15 实际接触前，相关左弯候选应出现 `predicted_collision` 或低间隙。
2. `danger_time_sec` 应位于轨迹中段，而非只报告终点。
3. `collision_part` 应指向左侧或左前相关边界。
4. 输出对应有限墙段，危险部位匹配左侧机身或左侧角点。
5. 同时输出旧 `0.413m` 首次触发记录，但 `legacy_only_explanation=false`。
6. 后方覆盖不足时，后退候选必须为 `UNKNOWN` 且不可选择。
7. 不得出现任何运动发布者。

本仓库环境没有 Round15 bag，不能在开发机伪造上述回放 PASS。

## 10. 后续验证门槛

本轮完成后仍不能直接自主过弯。按顺序推进：

1. **Round15 离线重放**：先证明碰板轨迹会被提前拒绝。
2. **运动模型标定**：选定步态后，自动短直行和短后退各重复 5 次，记录
   实际位移、横漂、Yaw、停车尾程和方差。
3. **安全链 Topic**：把真实 watchdog 和 estop 状态接入只读 Bool Topic，验证
   帧龄和故障语义。
4. **第一弯单次执行器**：另建受控制权管理的短段执行节点，每段结束必须
   `StopMove -> 等待稳定 -> 重建地图 -> 重新规划 -> 人工继续确认`。
5. 第一弯单次零接触后连续 3 次，才达到 B2.1 PASS。

第一弯 PASS 条件保持为：零接触、无人工修正、无 STALE、无 FAULT_STOP、
最终 Yaw 误差不超过 5 度、`CORRIDOR_REACQUIRE` 成功、`route=1/5`，并且同一
配置连续 3 次通过。

右弯、第二弯和完整五弯均不属于本轮实现。
