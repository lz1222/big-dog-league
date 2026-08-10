# LiDAR 数据源诊断结论 — 2026-08-07

## 排查手段

1. ✅ 搜索 `rk_inspection_ws/src` — 无 `/utlidar/cloud_base` 发布者
2. ✅ 搜索 `big-dog-league` — 无发布者（仅消费者脚本）
3. ✅ 搜索 `unitree_ros2/cyclonedds_ws/install` — 仅含消息类型定义，无可执行 bridge 节点
4. ✅ 搜索 `/opt/ros/foxy` — 无 LiDAR 相关 ROS 包
5. ✅ 搜索 `competition_non_arm.launch.py` 及所有子节点 — 无 LiDAR 发布者
6. ✅ 搜索 `go2_sdk_server_runtime.py` / `go2_sdk_udp_server` — 不含 LiDAR 逻辑
7. ✅ `systemctl list-units` — 仅 `unitree-upgrade.service`，无 LiDAR 服务
8. ✅ `dpkg -l` — 无 livox/hesai ROS2 驱动包
9. ✅ 全文件系统搜索 `cloud_base` 发布者 — 零结果
10. ✅ `~/.ros/log/` 发现 `hesai_lidar_node` 日志（2023年），确认历史上使用过 Hesai LiDAR ROS2 驱动

## 诊断结论

**`/utlidar/cloud_base` 的发布者不在当前任何代码库、安装包或系统服务中。**

### 最可能的两个场景

**场景 A（更可能）：Go2 固件直发 → ROS2 CycloneDDS 网络**

Go2 嵌入式控制器通过 CycloneDDS 将传感器数据（LiDAR/IMU/Odom）直接注入 ROS2 网络。数据在 ROS2 侧看起来像"普通的 topic 发布"，但实际发布者是机器人固件而非 ROS2 节点。

支持的证据：
- frame_id 为 `base_link`（机器人坐标系）
- 三个 topic（`/utlidar/cloud_base`, `/utlidar/robot_odom`, `/utlidar/imu`）同生命周期
- `ps aux` 从未显示 LiDAR 发布进程
- Go2 使用 CycloneDDS 作为其内部 DDS

**此场景下，数据消失的可能原因：**
- LiDAR 被 toggle OFF（需 `toggle_lidar.py ON --network eth0`）
- CycloneDDS 网络发现失败（配置变更后接口/域不匹配）
- 机器人固件重启或进入安全模式

**场景 B：独立 LiDAR ROS2 驱动进程（手动启动，已退出）**

存在一个外部 ROS2 驱动节点（Hesai/Livox），在之前的终端中手动启动，进程被杀后再未重启。

支持的证据：
- `~/.ros/log/` 中有 `hesai_lidar_node` 历史日志
- `/usr/local/lib/liblivox_lidar_sdk_*` 已安装
- Go2 的 LiDAR 可能是 Hesai 或 Livox 型号

**此场景下，数据消失的原因：**
- LiDAR 驱动进程被误杀
- 驱动需要手动启动且未被包含在任何 launch 文件中

---

## 下一步验证方案

```
步骤1: 确认 LiDAR 硬件开关
  python3 /home/unitree/toggle_lidar.py ON --network eth0

步骤2: 等 3 秒后检查 topic 列表
  source /opt/ros/foxy/setup.bash
  ros2 topic list 2>/dev/null | grep -E "cloud|lidar|utlidar|odom|imu"

步骤3: 若无数据，检查 Go2 固件 DDS 连接
  # 看 eth0 是否有 DDS 发现流量
  sudo tcpdump -i eth0 -n port 7400 or port 7410 -c 20

步骤4: 若仍无数据，尝试找 LiDAR 驱动
  # 检查是否安装了 Hesai/Livox ROS 驱动
  find /opt -name "*lidar*" -type f 2>/dev/null
  # 检查是否有 ROS2 LiDAR package
  source /opt/ros/foxy/setup.bash && ros2 pkg list | grep -i lidar

步骤5: 若以上都失败，Go2 可能通过 Unitree SDK DDS 发布传感器数据
  # 需要写一个 Python 脚本用 Unitree SDK 订阅 rt/utlidar/* 确认数据存在
```

---

## 数据链路完整图（当前已知）

```
Go2 LiDAR 硬件 ──→ [???] ──→ /utlidar/cloud_base (ROS2)
Go2 IMU         ──→ [???] ──→ /utlidar/imu (ROS2)
Go2 Odom        ──→ [???] ──→ /utlidar/robot_odom (ROS2)
                                  ↓
                          maze_full_auto.py 订阅
                                  ↓
                          计算 front / L / R / heading
                                  ↓
                          发布 /navigation/cmd_vel
                                  ↓
                       cmd_vel_udp_forwarder.py
                                  ↓
                          UDP 127.0.0.1:15001
                                  ↓
                       go2_sdk_udp_server (C++)
                                  ↓
                    Unitree SDK SportClient.Move()
                                  ↓
                            Go2 机器人运动
```

**断点：`[???]` → `/utlidar/cloud_base` 这一段。数据源的发现/启动机制不明。**
