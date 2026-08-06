# T0 LiDAR 静态卷尺测距标定

## 目的

校准 LiDAR 报出的距离与真实物理距离之间的偏差。

上一轮真机数据发现：LiDAR 报 1.20m vs 卷尺实测 0.53m（偏差 ~0.67m）。需要在多个距离点标定后修正。

## 前置条件

1. Go2 完全开机，3D SLAM 已激活（App 中启动）
2. 环境中有平整挡板（纸箱/木板，高约 0.5m）
3. 卷尺（≥2m）
4. DDS 话题可达（eth0 carrier=1）

## 步骤

### 1. 确认环境

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
ros2 topic list | grep utlidar
# 应看到: /utlidar/cloud_base, /utlidar/robot_odom, /utlidar/imu
```

### 2. 逐个距离录制

在每个距离，用卷尺精确测量从 **LiDAR 传感器表面** 到挡板的距离，然后录制：

```bash
cd ~/rk_inspection_ws
source /opt/ros/foxy/setup.bash

# 标定点 1: 0.30m
python3 tools/lidar_distance_calibration.py -d 0.30 -l "030cm_front"

# 标定点 2: 0.50m
python3 tools/lidar_distance_calibration.py -d 0.50 -l "050cm_front"

# 标定点 3: 0.80m
python3 tools/lidar_distance_calibration.py -d 0.80 -l "080cm_front"

# 标定点 4: 1.00m
python3 tools/lidar_distance_calibration.py -d 1.00 -l "100cm_front"

# 标定点 5: 1.50m
python3 tools/lidar_distance_calibration.py -d 1.50 -l "150cm_front"
```

每个点录制 15 秒，期间挡板保持不动。

### 3. 分析所有数据

```bash
python3 tools/lidar_distance_calibration.py --analyze
```

输出示例：
```
T0 LiDAR 标定分析 (5 个标定点)
  距离    LiDAR中值      偏移    偏移%      σ    点数   判定
 0.300      0.310    +0.010    +3.3%  0.005    200    OK
 0.500      0.515    +0.015    +3.0%  0.006    210    OK
 ...
标定结果:
  线性拟合:  LiDAR = 1.0234 × 实际 + 0.008m
  拟合 RMSE: 0.012m
  ✅  RMSE < 3cm — 达到验收标准
```

### 4. 应用标定参数

如果偏移 > 5cm，在 `src/rk_maze/config/lidar_distance.yaml` 中修改：

```yaml
lidar_distance:
  ros__parameters:
    # 根据标定结果调整:
    min_range_m: 0.08     # 保持不变（最近有效距离）
    max_range_m: 3.00     # 保持不变
    
    # 如果线性拟合结果 slope ≠ 1.0 或 intercept 显著:
    # 在 lidar_distance_core.py 中修正距离计算
```

### 5. 验收标准

| 指标 | 目标 | 可接受 |
|------|------|--------|
| RMSE | ≤ 3cm | ≤ 5cm |
| 偏移稳定度 | σ_偏移 ≤ 2cm | σ_偏移 ≤ 3cm |
| 单个标定点判定 | |offset| < 15cm |

## 已知问题

- LiDAR 原点与 base_link 原点存在 ~0.67m 的物理偏移
- 需要在代码中通过 `footprint_front_m` 或在距离计算中添加 `lidar_offset_x_m` 参数来修正
- 当前 `footprint_front_m=0.35` 已包含部分机身偏移，但仍不足以解释全部偏差
