# Unitree Go2 窄迷宫项目总计划

## 项目目标

在人工安全监护下，使用 `/utlidar/cloud_base`、`/utlidar/robot_odom`
和经过验证的 Go2 运动链完成窄迷宫自主避障。机器人只执行短轨迹，每段结束
必须 `StopMove -> 等待稳定 -> 重新感知 -> 重新规划`，禁止一次性下发完整
90 度转向。

最终路线为 `LEFT, LEFT, RIGHT, RIGHT, LEFT`。完整流程为巡线停止并释放旧
命令、迷宫取得控制权、完成五弯、出口停车、重新识别黑线、释放迷宫控制权。

## 固定硬件基线

- Go2 名义尺寸：长 `0.70m`、宽 `0.31m`。
- 挡板高度约 `0.45m`，最窄净宽约 `0.57m`。
- 点云：`/utlidar/cloud_base`，`PointCloud2`，`base_link`，约 `14.7Hz`。
- Odom：`/utlidar/robot_odom`，`Odometry`，`odom -> base_link`，约 `149Hz`。
- IMU：`/utlidar/imu`，`Imu`，约 `250Hz`。
- 当前灵动模式存在约 `0.25m/s` 起步门限、最短遥控点动约 `26.5cm`、固定
  左偏及静止 Yaw 漂移，未冻结为迷宫步态。

## 控制架构

正式运动链只能是：

```text
maze/gait module
  -> /control/locomotion_cmd
  -> command_mux_node
  -> /navigation/cmd_vel
  -> UDP forwarder
  -> Go2 SDK server
  -> Go2
```

`/navigation/cmd_vel` 的正式唯一 Publisher 必须是 `command_mux_node`。迷宫
节点不得直接发布该 Topic，不得把 `/api/sport/request` 作为正式绕行路径。

## 阶段及准入门

| 阶段 | 目标 | 进入下一阶段的硬门槛 |
|---|---|---|
| B0 | 硬件、DDS、传感器审计 | 点云、Odom、IMU 和 DDS 真机可用 |
| B1 | 五扇区感知 | 前向五区标定通过，无 STALE 漏检 |
| B1.5 | 运动桥安全基线 | Move、StopMove、两级 watchdog、急停通过 |
| B2.1-A | 第一弯局部几何 Dry Run | Round15 接触前被新几何拒绝，墙端和后向覆盖证据完整 |
| B2.1-B | 步态冻结与运动模型 | 同步态重复标定通过，模型状态 `VERIFIED` |
| B2.1-C | 自动短直行/短后退 | 人工逐次 ARM，各 5/5；后退可按覆盖结果禁用 |
| B2.1-D | 第一弯分段执行 | 每段 `ROBUST_SAFE`、StopMove、重扫、无接触 |
| B2.1-E | 第一弯验收 | 相同配置连续 3 次零接触 |
| B2.2-A | 独立右弯 | 右弯重新标定并连续 3 次通过 |
| B2.2-B | 五弯串联 | 顺序正确、每弯重扫、连续 3 次完整通过 |
| B3 | 巡线交接 | 控制权唯一、入口出口无残留命令 |
| B4 | 故障注入与封版 | 已测故障全部安全停车，比赛门槛通过 |

阶段不得跳过。更换步态后，运动比例、偏置、圆弧半径、尾程、动态足迹和
第一弯候选全部作废并重新标定。

## B2.1-A 当前设计要求

- 构建 `base_link` 周围可配置局部二维占据图。
- 独立输出前后八区的距离、有效点、覆盖和数据年龄。
- “没有后向点”必须解释为未知或覆盖不足，禁止解释为空旷。
- 提取有限墙线及墙端，同时保留降采样障碍点。
- 使用四向非对称动态矩形足迹。
- 分开配置步态横摆、点云、Odom、模型、物理目标间隙和停止尾程。
- 连续检查候选全过程、有限墙段、墙端和停止尾程。
- 输出最小间隙、最危险时间/姿态、墙段和足迹部位。
- 保留旧 `0.413m` 粗保护，但不得作为唯一碰撞解释。
- B2.1-A 最高只允许输出 `ROBUST_SAFE` 几何等级，禁止
  `EXECUTABLE_SAFE`。

## 候选安全等级

```text
UNSAFE
UNKNOWN
GEOMETRY_SAFE_UNCALIBRATED
NOMINAL_SAFE
ROBUST_SAFE
EXECUTABLE_SAFE
```

排序必须先比较等级，再最大化最小间隙和扰动鲁棒性，然后降低尾程、墙端、
Yaw 和横向误差风险，最后才比较时间。速度或路径长度不能抵消安全风险。

## 故障处理

点云/Odom/后向覆盖失效、watchdog、estop、无安全候选、预测碰撞、实际路径
偏差、Yaw 跳变、roll/pitch 超限、阶段超时和停止后未稳定都必须产生明确故障
原因。正式执行层必须立即 StopMove、持续零命令、清除旧指令、禁止自动恢复
旧运动并保存证据。

## 永久安全禁令

- 未经当前阶段和人工本轮授权，不发布非零速度、不 ARM、不自动执行矩阵。
- 禁止原地旋转候选 `vx=0, vy=0, wz!=0`。
- 禁止运动中切换步态。
- 禁止在 B2.1-A 执行第一弯、第二弯或完整迷宫。
- 禁止删除测试 bag、视频、日志、`build/install/log` 或用户修改。
- 禁止以降低安全门限作为碰撞的唯一修复。

## 工作记录规则

每次工作开始必须读取本文件、进度、验收矩阵、证据索引和
`config/maze_project_status.yaml`；结束时同步更新。没有真机或录包证据的检查
项必须写 `UNVERIFIED`。

