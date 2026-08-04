# 迷宫项目进度跟踪

## 当前状态

- 当前阶段：`B2.1-A`
- 当前子阶段：第一弯局部几何、墙端、扰动轨迹和 Round15 Dry Run
- 状态：`IN_PROGRESS`
- 自动运动：`禁止`
- 第一弯执行：`禁止`
- 第二弯及完整迷宫：`禁止`
- 当前 Git SHA：`cb7aa7a`

## 已完成

- B0 点云、Odom、IMU 和 DDS 基础审计。
- B1 前向五扇区感知和静态距离标定。
- B1.5 Move、StopMove、forwarder watchdog、SDK server watchdog、非法报文
  与遥控急停独立测试，历史记录为通过。
- 旧 B2 状态机可推进第一弯到 `route=1/5`。
- Round15 后补充的 `0.413m` 粗扫掠保护可在旧状态机重放约 `4.52deg`、
  `left≈0.362m` 时触发 `turn_sweep_unsafe`。
- B2.1-A 已实现局部占据图、后向三扇区、有限墙线/墙端、拟合置信度、动态
  矩形足迹、连续采样、停止尾程和六类第一左弯候选。
- 候选已使用六级结论和安全优先字典序，JSON 每帧输出排名前 5。
- Round15 回放器按历史点云快照和后续 Odom 实际路径检查连续足迹，并单独
  输出新几何与旧 `0.413m` 门限证据。
- 全部 98 项测试通过（包括原有旧测试和新 B2.1-A 测试）。
- Round15 v2 真包重放已执行（详见下方）。
- 真机构图性能：中位数约 5.2ms/帧，最大约 40ms/帧，14.65Hz 可实时。

## Round15 v2 真包重放结果 (2026-08-03)

- **bag**: `/home/unitree/maze_bags/b2_round15_20260802_144706`，存在且完整。
- **重放窗口**: 22s，322 张地图快照，3299 个 Odom 样本。
- **gate_status**: `FAIL`
- **最早几何报警**: 地图快照起始即触发（yaw≈−0.01°），危险部位 `right_side`，
  由右侧有限墙段 `wall_000` 的起始墙端 `(0.17, −0.29)` 距离机身右侧约 0.286m
  导致。该右侧墙端未匹配 Round15 实际碰撞（Round15 接触的是左侧）。
- **左侧匹配报警**: yaw≈23.07°，提前约 8.73s，预测净空 0.049m < 0.05m 目标，
  危险部位 `left_side` 与 Round15 实际左侧机身碰板一致。`dangerous_part_matches_round15=true`。
  但危险点未关联到有限墙段（`wall_segment=null`），属于孤立障碍点。
- **旧 0.413m 粗保护**: 本次重放中 `left` 扇区数据不可用，`legacy_0413_alert=null`。
- **后向覆盖**: `rear_coverage_insufficient`，三个后扇区均不可用。
- **全部前五候选**: `UNSAFE`，无安全候选。主要因为静止起步时右侧墙端过近。
- **gate 失败原因**: `matching_geometry_alert` 中的 `wall_segment` 为 null。

## 待解决的技术问题

1. **右侧墙端假阳性**: v2 重放中右侧墙 `wall_000` 起点 `(0.17, -0.29)` 位于
   base_link 右侧约 0.29m 处，导致静止时所有候选 UNSAFE。需核查该点是否为
   Go2 自身结构回波（右侧腿、侧板等），确认 body filter 是否覆盖充分。
2. **左侧障碍未关联墙段**: 左侧危险点 `(0.29, 0.35)` 预测净空 0.049m，
   代表内侧墙端，但未被提取为有限墙段。需审查 extract_wall_segments 在左侧
   的墙点密度和 minimum_points/minimum_length 是否匹配实际迷宫挡板回波模式。
3. **左侧右侧扇区不可用**: 重放窗口内 left 和 right 正侧扇区覆盖标记为 false，
   可能因为 B0 的 front_angle=20° 限制导致部分侧墙回波分类到斜前区。需验证
   front_angle 在窄通道（57cm）中对侧扇区覆盖的影响。
4. **B0 body filter 审计**: 当前 body_x_min/max 和 body_y_min/max 用于 B1，
   但 B2.1-A 的 LocalMapConfig 有独立的 body 和 front_leg 过滤。需统一
   基准或确认两套参数一致。

## 当前未完成

- Round15 v3 重放（修复上述问题后重新通过 gate）。
- 步态筛选和真实运动模型，属于 B2.1-B，当前不得提前运动。
- watchdog 和 estop 的只读 Bool 状态 Topic 真机确认。

## B2.1-A 静止真机感知验证结果 (2026-08-04)

基于 2026-08-03 录制的四个静态 rosbag 离线重放分析：

| 指标 | 目标 | 实测 | 判定 |
|---|---|---|---|
| 局部图频率 | ≥10Hz | 14.65-14.72Hz | **PASS** |
| 构图耗时中位数 | 实时 | 4.2-10.2ms | **PASS** |
| height_filtered_points | 记录 | map_statistics 中记录（186-357） | **PASS** |
| body_filtered_points | 记录 | map_statistics 中记录（70-98） | **PASS** |
| leg_self_filtered_points | 记录 | front_leg_self_filter_enabled=false，字段缺失 | **GAP** |
| 正前挡板可见 | 是 | front 扇区 valid，距离约1.27m | **PASS** |
| 两侧墙可见 | 是 | 2 wall_segments 检测到 | **PASS** |
| motion_output=false | 全帧 | 365+117 帧全部 False | **PASS** |
| execution_allowed=false | 全帧 | 365+117 帧全部 False | **PASS** |
| Twist/SDK 发布器 | 零 | 代码零引用，publisher_count=0 | **PASS** |
| 后向三扇区覆盖 | 记录 | rear/left_rear/right_rear 不可用，REVERSE 禁用 | **PASS (fail-closed)** |
| 前腿自回波伪碰撞 | 无 | 过滤器关闭，右侧碰撞来自 wall_endpoint | **待审计** |

静态 bag 清单：
- `b2_1_a_static_empty_20260803_004235`：182s，2679 点云帧，dry_run 输出完整
- `b2_1_a_static_maze_entry_20260803_011605`：137s，2020 点云帧，**无 dry_run 输出**
- `b2_1_a_static_maze_entry_retry_20260803_170707`：58s，863 点云帧，dry_run 输出完整

**关键发现**：
1. 迷宫入口静止时右脚印碰撞（collision_part=right_side，danger_geometry_type=wall_endpoint），危险点约 (0.25, -0.24)，与 Round15 v2 重放中 wall_000 起点 (0.17, -0.29) 模式一致。
2. 后方三扇区全部覆盖不足，REVERSE_SHORT 被正确禁用。
3. watchdog_ok=None、estop_triggered=None — 安全链 Topic 未接入。
4. front_leg_self_filter_enabled=false，腿自回波过滤未激活，依赖 body filter 矩形 (x:±0.40, y:±0.18) 过滤自身结构。

## 阻塞项

- v2 重放 gate FAIL，需修复左侧墙段关联和右侧假阳性问题后重新验证。
- **B2.1-A 静止验证**：右侧 wall_endpoint 在迷宫入口导致静态脚印碰撞，需审计 body filter 是否充分覆盖 Go2 右侧自身结构回波。
- 动态足迹、模型扰动和停止尾程尚未按冻结步态标定。
- 后向扇区没有真机点云数据支撑（Go2 默认前装 UT-LiDAR，后方视野受机器人自身遮挡）。

## 下一步唯一推荐任务

**审计右侧 wall_endpoint 静态碰撞根因，确认 body filter 是否需要调整。**
具体步骤：
1. 从迷宫入口静态 bag (`b2_1_a_static_maze_entry_retry_20260803_170707`) 提取原始点云，检查右侧 wall_000 起点附近的原始点是否来自 Go2 自身结构。
2. 如果属于自身回波，调整 body filter 的 body_y_min_m（当前 -0.18）或添加右腿局部过滤后重新验证。
3. 目标：静态脚印不再碰撞 → 进入 APPROACH_TURN 状态 → `current_footprint_safety.collision=false`。
4. 同步修复左侧障碍点 wall_segment 关联问题。

## 阶段结论

- 是否允许进入 B2.1-B：`否`
- 原因：B2.1-A 静止验证发现右侧 wall_endpoint 静态碰撞；左侧 wall_segment 关联未修复；
  后向覆盖 UNVERIFIED（物理限制）；安全链 Topic 未接入。
