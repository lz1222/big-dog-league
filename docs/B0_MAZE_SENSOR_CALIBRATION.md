# B0 Maze Sensor Calibration

## 1. 目的和安全边界

本流程只验证迷宫所需的点云区域距离和里程计姿态：

- `scripts/pointcloud_sector_monitor.py` 只订阅
  `/utlidar/cloud_base`。
- `scripts/odom_yaw_monitor.py` 只订阅
  `/utlidar/robot_odom`。
- 两个节点都不创建 publisher，不发布 `Twist`，不调用 Unitree API。
- 标定期间不要启动运动桥、command mux、gait Action Server 或自动任务。
- 需要改变朝向时，只能由现场人员使用遥控器低速操作，并始终保持急停可用。

本文所有 PASS/FAIL 单元格必须由真机测试人员填写。空白不表示通过。

## 2. ROS2 Foxy 环境

这两个监视器只使用标准 ROS2 消息，不需要 source Unitree 自定义消息
overlay。如果 `scripts/source_unitree_ros2.sh` 输出不存在的 Humble 或旧工作区
路径，先使用一个干净终端：

```bash
cd ~/big-dog-league

unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH PYTHONPATH

source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```

确认标准依赖和硬件 topics：

```bash
python3 -c "import rclpy, sensor_msgs_py, nav_msgs"
ros2 topic type /utlidar/cloud_base
ros2 topic type /utlidar/robot_odom
ros2 topic hz /utlidar/cloud_base
ros2 topic hz /utlidar/robot_odom
```

期望类型：

```text
/utlidar/cloud_base   sensor_msgs/msg/PointCloud2
/utlidar/robot_odom  nav_msgs/msg/Odometry
```

## 3. 运行监视器

终端 1：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

python3 scripts/pointcloud_sector_monitor.py \
  --ros-args \
  --params-file config/maze_sensor_monitor.yaml
```

终端 2：

```bash
cd ~/big-dog-league
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

python3 scripts/odom_yaw_monitor.py \
  --ros-args \
  --params-file config/maze_sensor_monitor.yaml
```

点云日志包括：

- `FRESH`/`STALE` 和消息 age。
- 接收帧率。
- frame id。
- 有效点数/原始点数。
- `front`、`left_front`、`right_front`、`left`、`right` 的百分位距离和点数。

里程计日志包括：

- `FRESH`/`STALE` 和消息 age。
- 接收帧率、frame 和 child frame。
- roll、pitch、yaw 的弧度和角度。
- 相对启动初始方向的累计转角 `turn`。
- 静止时长、漂移、漂移范围和漂移率。

## 4. 点云扇区定义

坐标遵循 `base_link`：`x` 向前、`y` 向左、`z` 向上。

`front_angle` 是每个扇区的角宽。默认 `45 deg` 时：

| 区域 | 角度范围 | 中心方向 | 最大量程参数 |
|---|---:|---:|---|
| `front` | `±front_angle/2` | `0 deg` | `front_max_range` |
| `left_front` | `front_angle/2 .. diagonal_angle_max` | 左斜前 | `front_max_range` |
| `right_front` | 对称负角度 | 右斜前 | `front_max_range` |
| `left` | `diagonal_angle_max .. side_angle_max` 或有效左斜前投影 | 左侧墙 | `side_max_range` |
| `right` | 对称负角度或有效右斜前投影 | 右侧墙 | `side_max_range` |

前方和斜前距离是有效径向距离的 `distance_percentile`。左右距离为正侧方
点或斜前墙点投影后的横向净距 `|y|` 百分位。这样可兼容真机对 45cm 挡板
缺少 `±90 deg` 回波的情况；后方点不参与这五个区域。

### 4.1 参数说明

| 参数 | 节点 | 单位/范围 | 说明 |
|---|---|---|---|
| `cloud_topic` | 点云 | topic 名称 | 点云输入，默认 `/utlidar/cloud_base` |
| `odom_topic` | 里程计 | topic 名称 | 里程计输入，默认 `/utlidar/robot_odom` |
| `z_min` / `z_max` | 点云 | m，`min < max` | 保留点的垂直范围 |
| `body_x_min` / `body_x_max` | 点云 | m，`min < max` | 自身过滤矩形的前后边界 |
| `body_y_min` / `body_y_max` | 点云 | m，`min < max` | 自身过滤矩形的左右边界 |
| `front_angle` | 点云 | deg，`0 < value <= 72` | 正前扇区总角宽 |
| `diagonal_angle_max` | 点云 | deg | 斜前扇区最大绝对角度 |
| `side_angle_max` | 点云 | deg | 正侧扇区最大绝对角度 |
| `min_range` | 点云 | m，`>= 0` | 丢弃离原点过近的点 |
| `front_max_range` | 点云 | m，`> 0` | 前、左前、右前最大距离 |
| `side_max_range` | 点云 | m，`> 0` | 左、右最大距离 |
| `distance_percentile` | 点云 | `%`，`0 < value <= 100` | 每个扇区的距离百分位数 |
| `side_projection_angle_min/max` | 点云 | deg | 侧墙斜前投影角度窗 |
| `side_projection_x_min/max` | 点云 | m | 侧墙投影前向 x 窗口 |
| `side_projection_min_x_span` | 点云 | m，`> 0` | 排除固定 x 前挡板的最小投影跨度 |
| `side_projection_lateral_tolerance` | 点云 | m，`> 0` | 同一投影侧墙簇允许的横向厚度 |
| `side_min_points` | 点云 | 正整数 | 单侧墙距离所需最少点数 |
| `side_continuity_tolerance` | 点云 | m，`>= 0` | 短墙候选延续已确认侧距允许的横向差值 |
| `stale_timeout` | 两者 | s，`> 0` | 超过此消息间隔后显示 `STALE` |
| `print_rate` | 两者 | Hz，`> 0` | 状态日志输出频率 |
| `stationary_linear_speed_threshold` | 里程计 | m/s，`>= 0` | 静止判定线速度上限 |
| `stationary_angular_speed_threshold` | 里程计 | rad/s，`>= 0` | 静止判定角速度上限 |
| `stationary_min_duration` | 里程计 | s，`>= 0` | 显示 `stationary=yes` 前的等待时间 |

`stale_timeout` 和 `print_rate` 在两个节点下分别配置；修改时应同时检查 YAML
中的两个节点块。程序启动时会拒绝非法范围，不会静默使用错误参数。

## 5. 五方向纸箱测试

### 5.1 准备

1. 机器人保持站立且完全静止，不启动任何运动节点。
2. 使用表面平整、尺寸已记录的纸箱。
3. 在地面标出相对 `base_link` 的 `0、+45、-45、+90、-90 deg` 射线。
4. 从 `base_link` 原点量到纸箱最近表面，记录实测距离。
5. 每个方向至少测试近、中、远三个距离；距离必须小于对应 max range。
6. 每个位置保持至少 10 秒，记录稳定距离、中位表现和点数。

建议在测试开始前先确定允许误差，例如：

```text
abs(monitor_distance - measured_distance) <= 0.10 m
```

允许误差必须在看结果之前确定，不能测试后为通过而放宽。

### 5.2 Front

1. 将纸箱中心放在 `0 deg` 射线上。
2. 确认 `front` 点数明显增加并输出有效距离。
3. 其他扇区不应出现同等强度的错误近距离。
4. 比较 `front` 与卷尺距离。

### 5.3 Left Front

1. 将纸箱中心放在 `+45 deg` 射线上。
2. 确认主要命中 `left_front`。
3. 检查没有左右镜像到 `right_front`。
4. 比较 `left_front` 与卷尺距离。

### 5.4 Right Front

1. 将纸箱中心放在 `-45 deg` 射线上。
2. 确认主要命中 `right_front`。
3. 检查没有左右镜像到 `left_front`。
4. 比较 `right_front` 与卷尺距离。

### 5.5 Left

1. 先将纸箱中心放在 `+90 deg` 射线上，检查直接侧向回波。
2. 再把纸箱作为平行左墙放在斜前投影窗口内。
3. 确认 `left` 输出的是纸箱到 `base_link` 中线的横向净距，不是斜距。
4. 检查点数至少达到 `side_min_points`。

### 5.6 Right

1. 先将纸箱中心放在 `-90 deg` 射线上，检查直接侧向回波。
2. 再把纸箱作为平行右墙放在斜前投影窗口内。
3. 确认 `right` 输出的是横向净距，并检查最小点数门槛。

## 6. 地面过滤标定

1. 清空机器人周围 `front_max_range` 内的物体。
2. 在 RViz 显示原始 `/utlidar/cloud_base`，Fixed Frame 设为 `base_link`。
3. 读取地面点的大致 z 分布，记录最高地面噪点。
4. 从保守的 `z_min` 开始运行监视器。
5. 逐步提高 `z_min`，直到空场时地面不再产生稳定的近距离扇区返回。
6. 放回纸箱，确认纸箱立面仍有足够点数，不能为了清除地面而把低矮障碍全部过滤。
7. 调整 `z_max`，排除不需要的高处结构，同时保留会撞到机身的障碍。
8. 将最终值和测试环境写入记录表。

地面过滤 PASS 必须同时满足：

- 空场没有持续的虚假近距离。
- 五方向纸箱仍能被正确检测。
- 点云消息持续 `FRESH`。

## 7. 机器狗自身点云过滤

1. 周围清空后，在 RViz 观察贴近 `base_link` 的固定点簇。
2. 测量这些固定点簇的 x/y 边界，不要直接使用机器人外形估计值。
3. 将边界加少量余量后写入
   `body_x_min/body_x_max/body_y_min/body_y_max`。
4. 运行监视器，确认自身点簇不再形成近距离。
5. 在 body filter 每条边界外放置纸箱，确认边界外障碍不会被误删。
6. 特别测试正前方 `body_x_max` 外侧；过滤区域过大可能掩盖即将碰撞的障碍。

自身过滤 PASS 必须同时满足：

- 空场自身点不会形成虚假障碍。
- body filter 外的纸箱仍可检测。
- 参数边界有实测依据并已记录。

## 8. 静止 Yaw 漂移测试

1. 将机器人放在不会振动的水平地面。
2. 不启动运动节点，不触碰机器人。
3. 启动 `odom_yaw_monitor.py`，等待状态从 `settling` 变为 `yes`。
4. 连续记录至少 10 分钟。
5. 记录初始 yaw、最终漂移、最大漂移范围和 `deg/min`。
6. 如果日志持续显示 `stationary=no`，先根据静止 odometry 噪声调整两项
   stationary threshold，再重新开始完整测试。
7. 测试中发生移动或碰撞后，本轮静止统计会重置，必须重新计时。

PASS 阈值应由迷宫允许的转向误差预先确定。未确定阈值时只能记录数据，不能
填写 PASS。

## 9. 左右转 Yaw 符号测试

1. 现场人员使用遥控器低速左转约 `+90 deg`，停止并记录：
   - raw yaw 变化方向。
   - 累计 `turn` 的符号和角度。
2. 回到初始朝向，确认累计角变化与实际旋转方向一致。
3. 使用遥控器低速右转约 `-90 deg`，重复记录。
4. 再跨越一次 `+pi/-pi` 边界，确认 raw yaw 跳变时累计 `turn` 连续，没有
   约 `360 deg` 的错误突变。

按标准 ROS `base_link` 约定，左转通常为正、右转通常为负，但必须以本机实测
结果为准。发现相反时先检查 frame 和里程计定义，不要直接在上层硬编码取反。

## 10. PASS/FAIL 记录表

### 10.1 基础接口

| 项目 | 期望 | 实测 | PASS/FAIL | 日期/人员 | 备注 |
|---|---|---|---|---|---|
| Cloud topic/type | `/utlidar/cloud_base`, `PointCloud2` |  |  |  |  |
| Cloud frame | `base_link` |  |  |  |  |
| Cloud frequency | 记录并满足控制需求 |  |  |  |  |
| Odom topic/type | `/utlidar/robot_odom`, `Odometry` |  |  |  |  |
| Odom frames | `odom -> base_link` |  |  |  |  |
| Odom frequency | 记录并满足控制需求 |  |  |  |  |
| Cloud stale detection | 停流后超过阈值显示 `STALE` |  |  |  |  |
| Odom stale detection | 停流后超过阈值显示 `STALE` |  |  |  |  |

### 10.2 五方向距离

| 方向 | 纸箱实测距离 | Monitor 距离 | 误差 | 点数 | PASS/FAIL | 备注 |
|---|---:|---:|---:|---:|---|---|
| Front |  |  |  |  |  |  |
| Left Front |  |  |  |  |  |  |
| Right Front |  |  |  |  |  |  |
| Left |  |  |  |  |  |  |
| Right |  |  |  |  |  |  |

### 10.3 过滤参数

| 项目 | 最终参数/结果 | PASS/FAIL | 日期/人员 | 备注 |
|---|---|---|---|---|
| `z_min` |  |  |  |  |
| `z_max` |  |  |  |  |
| `body_x_min/max` |  |  |  |  |
| `body_y_min/max` |  |  |  |  |
| 空场虚假近距离 |  |  |  |  |
| 边界外纸箱保留 |  |  |  |  |

### 10.4 Yaw

| 项目 | 期望/阈值 | 实测 | PASS/FAIL | 日期/人员 | 备注 |
|---|---|---|---|---|---|
| 10 分钟静止漂移 | 测试前填写 |  |  |  |  |
| 静止漂移率 | 测试前填写 |  |  |  |  |
| 左转符号 | 记录实际约定 |  |  |  |  |
| 右转符号 | 与左转相反 |  |  |  |  |
| `+pi/-pi` 跨界 | 累计转角连续 |  |  |  |  |
| 左转 90 deg 误差 | 测试前填写 |  |  |  |  |
| 右转 90 deg 误差 | 测试前填写 |  |  |  |  |

## 11. 尚未确认

完成本文档前，以下项目必须保持“未确认”：

- 当前 YAML 中 z 和 body filter 默认值是否适合真机。
- 五个区域的实际距离误差和最优百分位数。
- 小纸箱在各距离下的最低有效点数。
- 点云和 odometry 的长时间丢帧、延迟和 stale 行为。
- 静止 yaw 漂移和温度变化影响。
- 左右转符号、90 度转向误差及正负 pi 跨界行为。
- 这些传感器结果尚未接入任何运动控制或安全停止链。
