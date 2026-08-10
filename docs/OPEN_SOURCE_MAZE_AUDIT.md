# GO2_OPEN_SOURCE_MAZE_MIGRATION_V1：Phase A 参考仓库审计

审计日期：2026-08-08。范围严格限于只读克隆、源码/launch/config/许可证检查和文档；没有
运行 `system_real_robot.sh`、校准程序或任何运动节点，没有发布非零 `/navigation/cmd_vel`
或 `/api/sport/request`。

## 结论先行

当前固定迷宫应继续采用：

```text
/utlidar/cloud_base (base_link) + /utlidar/robot_odom + /utlidar/imu
  -> 现有 SensorSnapshot / 感知 / L-L-R-R-L 状态机
  -> /navigation/cmd_vel -> cmd_vel_udp_forwarder -> UDP
  -> go2_sdk_udp_server -> SportClient.Move()
```

不替换状态机，不启用完整 Point-LIO，不接入 `/api/sport/request`，不改 DDS Domain，也不引入
Nav2/FAR Planner。下一阶段唯一建议是 **PHASE B：只读 DDS/LiDAR adapter 设计与测试先行**；
先修复和量化 Cloud gap，再讨论感知算法。

## 当前工作区基线

- 分支：`maze-fusion-t0-t3`；HEAD：`2a14e81 fix(bringup): use validated USB line camera profile`。
- 工作区在审计开始前已有大量修改和未跟踪文件；均已保留，未 reset/clean/checkout/commit。
- 平台实测：`aarch64`、6 logical CPU、约 7.2 GiB RAM（当时可用约 2.1 GiB）。完整 LIO +
  地图不应在无性能证据下加入常驻链。
- 既有项目已在 ROS Domain 0 发现 Go2 裸 DDS 传感器发布者；当前 dry run 的 v2 订阅
  `/utlidar/cloud_base`、`/utlidar/robot_odom`、`/utlidar/imu`，并默认零输出。

## 参考仓库可复现清单

克隆位置均为 `/home/unitree/go2_open_source_reference`，不在本工作区 `src/` 中。

| 仓库 | 分支 / tag | SHA | 最近提交 | ROS / 硬件定位 | 许可证结论 |
|---|---|---|---|---|---|
| `jizhang-cmu/autonomy_stack_go2` | `foxy-humble` | `43d5f54b389b251713f0097893c30fa76c870d54` | 2026-02-18 `update download link` | README 声称 Ubuntu 20.04 + Foxy 与 Go2 EDU/L1；同时含 Humble | 根 LICENSE 缺失，禁止源码复制 |
| `unitreerobotics/unitree_ros2` | `master` | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` | 2026-07-02 `chore(h2): update h2_loco api` | 官方 Go2/B2/H1 ROS2 DDS；README 明示 Ubuntu 20.04/Foxy | BSD-3-Clause |
| `unitreerobotics/point_lio_unilidar` | `main`, `v2.0.2` | `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` | 2025-06-05 `update to v2.0.2` | README 明示 Ubuntu 20.04 + **ROS Noetic/catkin**，L1/L2 | GPL-2.0，禁止直接并入 |

已查看的主要目录：

```text
autonomy_stack_go2/
  system_real_robot*.sh
  src/base_autonomy/{local_planner,terrain_analysis,sensor_scan_generation,...}
  src/slam/point_lio_unilidar/{launch,config,src}
  src/utilities/{transform_sensors,unitree_pkgs}
  src/route_planner/{far_planner,boundary_handler,graph_decoder}
unitree_ros2/
  cyclonedds_ws/src/{cyclonedds.xml,unitree/unitree_{go,api}}
  example/src/src/{go2,read_low_state.cpp,read_motion_state.cpp}
point_lio_unilidar/
  {CMakeLists.txt,package.xml,config/unilidar_l1.yaml,launch,src,include}
```

## A. ROS/DDS 与真实启动依赖图

### 官方事实来源：`unitree_ros2`

- README 明确 SDK2/Go2 使用 CycloneDDS；Foxy 建议 `rmw_cyclonedds_cpp`，并要求 CycloneDDS
  `0.10.x`。其 `setup*.sh` 仅绑定网卡的 `CYCLONEDDS_URI`，XML 中 `Domain Id="any"`，**没有
  Domain 10 的要求**。
- 官方示例说明 ROS2 消息可直接参与机器人 DDS；官方 RViz 示例直接发现
  `/utlidar/cloud`，frame 为 `utlidar_lidar`。这与本机实测的
  `_CREATED_BY_BARE_DDS_APP_` 发布者相一致：传感器是车端裸 DDS 可发现端点，并非本机
  `lidar_dds_bridge.py` 的产物。
- 官方控制为 `unitree_api::msg::Request` 发布至 `/api/sport/request`；`SportClient.Move`
  只是填充这个 Request。此路径与当前 UDP SDK bridge 是两条最终控制出口，不能并存。

### `autonomy_stack_go2` 的真实链

`system_real_robot.sh` 仅 source install 后启动 `vehicle_simulator/system_real_robot.launch`：

```text
transform_sensors: /utlidar/cloud + /utlidar/imu
  -> /utlidar/transformed_cloud + /utlidar/transformed_imu
point_lio_unilidar(mapping_utlidar.launch)
  -> /registered_scan + /state_estimation
terrain_analysis -> /terrain_map
localPlanner -> /path
pathFollower -> /cmd_vel 且 is_real_robot=true 时 -> /api/sport/request
```

该 launch 还启动 joystick 与 RViz，默认 local planner `useTerrainAnalysis=true`。其
`unitree_setup.sh` 已改为 source Humble，和本项目 Foxy 不可直接照搬；配置中固定
`enp3s0`，不适合当前实际网卡/Domain 0 环境。

### DDS 决策

保留现有 `ROS_DOMAIN_ID=0` 和单一、显式的 CycloneDDS 运行环境。未来 adapter 应检查：
RMW、Domain、`CYCLONEDDS_URI` 解析性、实际加载的 `libddsc`、网卡 UP 与三传感器 QoS/收包；
不能复制含新 XML 元素的配置，也不能用 `Domain=10` 隔离 forwarder。

## B. LiDAR、IMU、TF 与 Odom 比较

| 项目 | 当前迷宫链 | autonomy_stack_go2 | unitree_ros2 | 独立 point_lio_unilidar |
|---|---|---|---|---|
| LiDAR 输入 | `/utlidar/cloud_base`，实测 `base_link` | `/utlidar/cloud` 后固定旋转、去机身框、改写为 `body` | `/utlidar/cloud`，文档 frame `utlidar_lidar` | `/unilidar/cloud`（参数化） |
| IMU 输入 | `/utlidar/imu` | `/utlidar/imu` 后转换/偏置补偿 | `LowState.imu_state` 与官方状态消息；未定义 UTLiDAR 转换 | `/unilidar/imu`（参数化） |
| Odom | `/utlidar/robot_odom` | Point-LIO `/state_estimation` | `SportModeState.position/velocity` 仅官方状态来源之一 | Point-LIO `/pointlio/odom` |
| frame/TF | `base_link` 点云可直接做局部迷宫 | `body -> utlidar_lidar_1` 使用硬编码约 165° pitch 与时间补偿；LIO 输出 `camera_init -> aft_mapped` | 官方示例仅说明 `utlidar_lidar` | ROS1 输出 `camera_init -> aft_mapped` |

`autonomy_stack_go2/transform_everything.py` 在 Python callback 内完整解码、旋转、删点、重建
PointCloud2，并使用安装/标定假设（165° pitch、15.1° IMU 修正、`~/Desktop/imu_calib_data.yaml`）。
这不是可直接移植的通用 adapter：它会重引入 callback backlog 和未经本机验证的外参。

**选择 `cloud_base`。** 当前数据已实测其 frame 为 `base_link`，满足局部墙/开口、动态足迹和
零 TF 依赖的需求。`/utlidar/cloud + TF` 仅作为未来 adapter 的 fallback：必须先记录静态外参、
时间戳、点字段和与 `cloud_base` 的逐帧一致性，再允许切换。`cloud_deskewed` 也只可被记录/比较，
不可假定来源或语义。

## C. Point-LIO 审计

独立仓库是 ROS1（`catkin`, `roscpp`, `tf`）工程，`CMakeLists.txt` 启用 OpenMP、PCL、PythonLibs、
IKD-tree，使用 200000 深度订阅队列和 lidar/IMU deque；每帧执行 IMU 同步、去畸变、EKF、最近邻
匹配、增量地图，并可发布 cloud/map/odom/path。其 YAML 支持 `time_lag_imu_to_lidar`、时间单位、
盲区、外参、IMU 初始化与按 IMU 频率传播。这些是值得借鉴的**接口契约**：单调时间、回退检测、
队列上限、Cloud/IMU 时间配对、空数据处理、可观测耗时。

但它不适合默认置入本链：GPL-2.0、ROS1→Foxy 迁移、frame 不兼容、独立地图/odom 会与
`/utlidar/robot_odom` 形成双来源，且本机只有约 2.1 GiB 可用 RAM。完整 Point-LIO 结论：
**REJECT（当前运行链）**；只以 `REFERENCE_ONLY` 借鉴 timestamp/deskew/TF 验证方法。若未来要重启
该决策，先在离线 bag 做 cloud_base 与 deskewed/Point-LIO Odom 的时间、漂移、CPU、RAM、丢帧对比，
并完成许可证与独立 ROS2 实现审查。

## D. Local Planner、Path Follower 与碰撞模型

`autonomy_stack_go2/localPlanner.cpp` 并非全局规划器：它离线读取 343 条 template path、7 个
起始路径组和 voxel-to-path correspondence；运行时以 10° 方向离散（36 rotations）旋转轨迹模板。
输入为 `/state_estimation`、`/registered_scan` 或 `/terrain_map`、`/way_point`、边界、手柄；障碍点
先下采样，再按对应表为路径累积 blocked point 或高度 penalty。可行条件为每条路径障碍点数小于
`pointPerPathThre`，分数结合目标方向、路径末端方向、旋转方向权重、地形 penalty；无路径时发布
仅含原点的 `/path`。

它的 vehicle footprint 是矩形 `vehicleLength × vehicleWidth`（real launch: `0.3 × 0.7 m`），
主要碰撞判断通过预计算 path sweep/correspondence 进行；`checkRotObstacle` 默认 false，且没有当前
项目所需的 swept-footprint stop-tail、传感器 stale 或 JointHealthGuard。

`pathFollower.cpp` 是 look-ahead 跟踪器：从 `/path` 选 `lookAheadDis` 前视点，以 heading error 乘
yaw gain、夹到 `maxYawRate`，并以 `maxAccel/100` 每 100 Hz 改速；终点减速、停止位、倾斜限速和
bitwise `/stop` 都存在。危险点是 two-way 模式会改成负速度，且 real launch 的
`is_real_robot=true` 会在同一循环发布 `/cmd_vel` **和** `/api/sport/request`。因此只可借鉴
轨迹评分、前视/heading、加速度/rate clamp 的设计，不可直接启用。

## E. Terrain、Waypoint、FAR Planner 与安全

- `terrain_analysis` 是累积式地形 voxel/时间衰减、量化高度、动态障碍与 no-data obstacle 系统，
  适合较大空间但参数众多、与固定向下 L1/窄 0.60 m 走廊和当前每帧局部网格不匹配。**REJECT**。
- waypoint 示例依赖 RViz/人工点击与导航边界；可借鉴“局部目标”消息语义，但 L-L-R-R-L 的目标必须
  由状态机产生。**REFERENCE_ONLY**。
- FAR Planner 使用 visibility graph、未知空间探索、boundary/graph decoder，超出已知固定拓扑。
  **REJECT**。
- 三个开源实现都没有覆盖当前要求的 Cloud/Odom/IMU stale 锁存恢复、JointHealthGuard、无后方感知
  不倒车、OpeningEvidence expected-side 检查。因此现有安全状态机不能替换；所有候选算法必须被其
  零输出、stale、紧急停止门控包裹。

## 模块评分矩阵

| 模块 | 来源 | 成熟度 | Foxy | Go2 L1 | 迷宫适配 | CPU | 风险 | 建议 |
|---|---|---|---|---|---|---|---|---|
| DDS 环境事实 | unitree_ros2 | HIGH | HIGH | HIGH | HIGH | LOW | LOW | ADAPT |
| LiDAR subscriber/adaptor 思路 | autonomy + 官方 | MEDIUM | MEDIUM | MEDIUM | HIGH | LOW | MEDIUM | ADAPT |
| 完整 Point-LIO | point_lio | HIGH | LOW（ROS1） | HIGH | LOW | HIGH | HIGH/GPL | REJECT |
| timestamp/IMU 配对方法 | point_lio | HIGH | N/A | HIGH | HIGH | LOW | LOW（独立实现） | REFERENCE_ONLY |
| IMU 硬编码转换 | autonomy | MEDIUM | HIGH | LOW（外参未知） | LOW | MEDIUM | HIGH | REJECT |
| TF 管理原则 | 官方 + point_lio | MEDIUM | HIGH | MEDIUM | HIGH | LOW | MEDIUM | ADAPT |
| 模板轨迹 / correspondence | autonomy local_planner | HIGH | HIGH | N/A | MEDIUM | LOW | MEDIUM/授权 | REFERENCE_ONLY |
| Path follower 前视/限幅 | autonomy | MEDIUM | HIGH | N/A | HIGH | LOW | HIGH（双出口/倒车） | REFERENCE_ONLY |
| Terrain analysis | autonomy | MEDIUM | HIGH | MEDIUM | LOW | HIGH | HIGH | REJECT |
| Waypoint | autonomy | MEDIUM | HIGH | N/A | LOW | LOW | MEDIUM | REFERENCE_ONLY |
| FAR planner | autonomy | MEDIUM | HIGH | N/A | LOW | HIGH | HIGH | REJECT |
| Sport Request 控制 | unitree_ros2 | HIGH | HIGH | HIGH | LOW | LOW | CRITICAL（双出口） | REJECT |
| 点云预处理接口/限时观测 | point_lio | HIGH | N/A | HIGH | HIGH | LOW | LOW（独立实现） | REFERENCE_ONLY |
| 当前 swept collision checker | 当前项目 | HIGH | HIGH | HIGH | HIGH | LOW | LOW | 保留 |

## Migration Candidate Table

| ID | Source repo | Source module | Function | Target module | Expected benefit | Dependencies | Risk | License | Decision |
|---|---|---|---|---|---|---|---|---|---|
| M01 | unitree_ros2 | `setup.sh`, `cyclonedds.xml`, README | RMW/网卡/DDS事实校验 | `tools/check_dds_environment.sh`（待建） | 同域同库可观测 | Foxy CycloneDDS | 不能照抄固定网卡/XML | BSD-3 | ADAPT |
| M02 | 当前实测 + 官方 | `/utlidar/cloud` 文档 | topic/frame fallback 契约 | `rk_maze/lidar_input_adapter.py`（待建） | cloud_base 优先、cloud fallback 可诊断 | rclpy/TF2 可选 | frame/外参未经验证 | 事实，不复制 | ADAPT |
| M03 | point_lio | `parameters.cpp`, sync callback | 时间单调、pairing、gap 观测思想 | 现有 SensorSnapshot/新测试 | 暴露断流与时间错配 | 无新增依赖 | 不复制 GPL 表达 | GPL-2（思想） | REFERENCE_ONLY |
| M04 | autonomy | `localPlanner.cpp` | 离线轨迹库 + clearance 评分思想 | `rk_maze/trajectory_library.py`（待建） | 可解释的转弯候选比较 | 当前 grid/footprint | 授权不清、尺寸不同 | 未明确 | REFERENCE_ONLY |
| M05 | autonomy | `pathFollower.cpp` | look-ahead、wz/加速度限幅思想 | `rk_maze/trajectory_follower.py`（待建） | 平滑 Dry Run 指令生成 | 当前安全状态机 | 原实现倒车/直控 | 未明确 | REFERENCE_ONLY |
| M06 | autonomy | `transform_everything.py` | 硬编码 L1 转换 | 无 | 无可验证收益 | 标定文件 | 外参、callback负载 | 未明确 | REJECT |
| M07 | point_lio | full mapping | LIO/map/odom | 无 | 地图能力 | ROS1, PCL, OpenMP | GPL、内存、双 odom | GPL-2 | REJECT |
| M08 | autonomy | terrain/FAR | 地形与全局探索 | 无 | 固定迷宫无明确收益 | 多节点 | 高 CPU、任务偏离 | 未明确 | REJECT |
| M09 | unitree_ros2 | `ros2_sport_client` | `/api/sport/request` | 无 | 无；现链已完整 | unitree_api | 第二运动出口 | BSD-3 | REJECT |

## 性能、重编译与测试计划

- 无需为 Phase A 重编译 workspace；本轮仅增加 Markdown。Phase B 若新增纯 Python adapter/测试，也不
  需要 colcon；若引入 C++/消息/TF 包才需明确依赖后做选择性 rebuild。
- 性能预算：LiDAR 感知须跟上约 15 Hz，planner 10–15 Hz，control 20 Hz；每模块记录
  `perception_ms/planning_ms/control_ms`、RSS/CPU、Cloud max gap、队列深度。不可接受 callback backlog。
- 测试顺序（均 Dry Run/Replay，发布命令必须为零）：
  1. 60 s 与 10 min 只读 Domain 0 传感器探针，记录 cloud gap、`/utlidar/lidar_state` 与网卡计数器；
  2. 离线 adapter 单测：cloud_base 优先、cloud fallback、frame mismatch、空/NaN/stale、QoS/Domain；
  3. synthetic/replay：时间回退、Cloud/IMU 配对、墙/ground/opening/候选轨迹、无安全轨迹；
  4. v2 长时 Dry Run：确认 state、opening、candidate、selected trajectory 可观测且实际 cmd 恒为零。
- 未来真实测试只可另行提交计划（R0 静止传感器→人工推动→单段→单弯…），不在本轮执行。

## 明确回答

1. 是否需要改变 DDS Domain：**NO**，继续 Domain 0；只统一各进程环境与加载库。
2. 是否需要 Point-LIO：**NO**，当前拒绝完整部署。
3. 是否继续 `/utlidar/cloud_base`：**YES**，作为默认；`/utlidar/cloud + TF` 仅为经验证 fallback。
4. 是否改变 L-L-R-R-L 状态机：**NO**。
5. 是否改变 `/navigation/cmd_vel -> UDP -> SDK`：**NO**。
6. 本轮是否发送真实非零运动：**NO**。
7. 下一阶段唯一推荐：**PHASE B，DDS/LiDAR adapter 的测试先行设计；先解决/量化 Cloud gap。**

