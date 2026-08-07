# Unitree Go2 窄迷宫项目交接审计

## 审计范围与真实性边界

本文件用于把迷宫项目交接给 Claude Code。本轮只读取仓库、rosbag 和纯逻辑代码；未启动 ROS 节点、运动桥或任何实体运动。

审计执行工作区是 /home/lqsdaba/daihao1/big-dog-league，不是已确认的 Go2 本机目录。Go2 预期仓库路径为 /home/unitree/big-dog-league。接手时必须先执行 pwd、git status --short 和 git rev-parse HEAD，不得假设两处内容一致。

项目目标是在人工监护下让 Go2 通过 LEFT、LEFT、RIGHT、RIGHT、LEFT 窄迷宫。名义机身为 0.70m x 0.31m，最窄净宽约 0.57m，挡板约 0.45m 高。正式执行必须是人工逐次确认的短段：短段 -> StopMove -> 实体稳定 -> 重扫 -> 重规划。入口和出口巡线交接属于 B3，当前不实现。

## 当前项目状态

- 正式阶段：B2.1-A。
- 子阶段：第一弯局部几何、墙端、扰动轨迹与 Round15 只读重放。
- 自动运动、第一弯执行、第二弯和完整迷宫：均为禁止。
- config/maze_project_status.yaml 的下一门槛是 b2_1_a_rear_coverage_and_local_map_hardware_validation。
- 当前唯一下一任务：B2.1-A 静止真机感知验证。验证后向三扇区覆盖或明确不覆盖时的 fail-closed 行为、局部图频率、地面过滤和前腿自身点云过滤；保存只读 rosbag。不得发送非零运动。

### Git 审计快照

最终审计时分支为 master，HEAD 与 origin/master 均为 c220f11a92d3eba7be479ab667e7044f4f30edb7（c220f11），远端为 ssh://git@ssh.github.com:443/lz1222/big-dog-league.git，二者没有提交差异。

审计过程中观察到 HEAD 从 16251ae 变为 c220f11；本轮没有执行任何 Git 写操作，变化视为外部操作。最终工作区只有本轮新建但未跟踪的 docs/CLAUDE_CODE_MAZE_HANDOFF.md 与 docs/CLAUDE_CODE_FIRST_PROMPT.md。evidence/ 已由当前 HEAD 跟踪。接手时仍须先报告实际差异；不得通过 reset、restore、clean 或同步覆盖清除任何已有工作。

## 代码与系统地图

| 模块 | 状态 | 说明 |
|---|---|---|
| scripts/pointcloud_sector_monitor.py | IMPLEMENTED | B0 点云五扇区只读监控。 |
| scripts/odom_yaw_monitor.py | IMPLEMENTED | RPY、Yaw unwrap、累计转角和静止漂移只读监控。 |
| scripts/maze_perception_core.py、scripts/maze_perception_dry_run.py | IMPLEMENTED | B1 五扇区、freshness、滞回和 JSON 建议；不运动。 |
| scripts/maze_navigation_core.py、scripts/maze_navigation_dry_run.py | DRY_RUN_PASS | 旧 B2 固定五弯策略和人工干跑诊断。 |
| scripts/maze_first_turn_core.py | UNIT_TEST_PASS | B2.1-A 局部图、八扇区、有限墙段/端点、动态足迹、连续扫掠和六类第一左弯候选。 |
| scripts/maze_first_turn_dry_run.py | UNIT_TEST_PASS | 只读 ROS2 节点，唯一业务输出是 JSON String。 |
| scripts/maze_round15_bag_analyzer.py、scripts/maze_round15_replay_core.py | DRY_RUN_PASS | 仅读取 SQLite rosbag，按历史局部图和后续实际 Odom 回放。 |

maze_first_turn_dry_run.py 不导入 Unitree SDK，不创建 Twist Publisher，不调用 Move、StopMove 或 /api/sport/request。它订阅点云、Odom、watchdog/estop Bool，唯一业务发布器为 /maze/first_turn/dry_run_status，并固定输出 motion_output=false、execution_allowed=false、safety_action=STOP_MOVE_REQUIRED。

当前 HEAD 的 B2.1-A 代码含默认关闭的 front_leg_self_filter_enabled 和左右前腿局部过滤矩形。它只用于 B2.1-A 静止标定，不能以扩大 body_x/body_y 整体矩形替代，因为整体矩形会遮蔽机头正前方障碍。尚未获得真机验收。

### 运动链，存在但当前禁止启动

~~~text
maze/gait module -> /control/locomotion_cmd -> command_mux_node
-> /navigation/cmd_vel -> cmd_vel_udp_forwarder.py
-> UDP 127.0.0.1:15001 -> go2_sdk_udp_server -> SportClient.Move/StopMove
~~~

相关源文件：src/rk_safety/rk_safety/command_mux_node.py、src/rk_go2_sdk_bridge/scripts/cmd_vel_udp_forwarder.py、src/rk_go2_sdk_bridge/src/go2_sdk_udp_server.cpp、src/rk_go2_sdk_bridge/launch/go2_sdk_udp_bridge.launch.py、src/rk_locomotion/rk_locomotion/gait_control_node.py。

启动 go2_sdk_udp_bridge.launch.py、go2_sdk_udp_server、gait_control_node 的真实模式，或向 /navigation/cmd_vel 发布非零 Twist，均为 RISK_REQUIRES_HUMAN_REVIEW。B2.1-A 不得运行它们。

## 传感器与硬件事实

历史文档与 rosbag 支持以下输入：

~~~text
/utlidar/cloud_base  sensor_msgs/msg/PointCloud2  frame=base_link  约14.7Hz
/utlidar/robot_odom  nav_msgs/msg/Odometry        odom -> base_link 约149Hz
/utlidar/imu         sensor_msgs/msg/Imu          约250Hz（历史单次记录）
~~~

静止 rosbag：

- evidence/static/b2_1_a_static_empty_20260803_004235/：182.354 秒，点云 2679、Odom 27227、B2.1 状态 365。
- evidence/static/b2_1_a_static_maze_entry_retry_20260803_170707/：58.751 秒，点云 863、Odom 8801、B2.1 状态 117。

现有静止证据表明中央 rear 覆盖不足；代码将其标为 rear_coverage_insufficient，而不是空旷空间。后退候选因此必须保持 UNKNOWN，自动后退禁止。left_rear/right_rear 不能因前方数据新鲜而被推定为可用。

## Round15 事故与回放

### 已确认的物理事故

ROUND15_PHYSICAL 的数据库是 evidence/round15/b2_round15_20260802_144706/b2_round15_20260802_144706_0.db3，SHA256 为 6788e8687343471cfae44d008a316ad271b2677ce233e5c303103e85fa7b3b19。

物理结论为 FAIL：第一左弯约 44deg 时左侧机身轻触转弯内侧板。事故附近历史记录为 front=0.487m、left_front=0.647m、left=0.393m、right=0.263m；转前人工间隙左约 15/17cm、右约 24/22cm，转后左约 9/9cm、右约 27/27cm。旧 B2 可推进 route=1/5，但该策略结果不能抵消物理接触。

旧 B2 的粗门限在 scripts/maze_navigation_core.py 中以外接圆半径计算：sqrt((0.70/2)^2 + (0.31/2)^2) + 0.03 约等于 0.413m。config/maze_navigation_dry_run.yaml 的 turn_open_distance_m=0.42 留出该边界；历史重放在约 4.52deg、left 约 0.362m 触发 turn_sweep_unsafe。这是 DRY_RUN_PASS，不是物理首弯通过。

### 当前 B2.1-A 重放结论

审计本轮重新只读运行 scripts/maze_round15_bag_analyzer.py，输出到 /tmp/claude_handoff_round15_replay_current.json。结果是 gate_status=DRY_RUN_PASS、physical_contact_result=FAIL、motion_output=false、publisher_count=0。

匹配 Round15 左侧危险的合格审计片段在预测约 23.980deg、接触前约 7.278s 提示 left_side，但该片段是 wall_fragment，明确 reliable_for_turn_model=false；不能据此授权运动。全局最小预测净空仍为 0m，当时没有 ROBUST_SAFE 候选，后方覆盖不足。

仓库另有 evidence/round15/.../b2_1_a_round15_geometry_summary.json 与 geometry_summary_v2.json，二者写为 gate_status=FAIL，字段与当前 HEAD 的短片段证据逻辑不同。这是证据版本差异，必须保留并由 Claude 在任何改动前复现、记录代码、配置和输出 SHA；不得删除旧摘要，也不得选择对结论更有利的版本。

## 接管状态表

| 项目 | 当前状态 | 证据 | 最近结果 | 阻塞问题 |
|---|---|---|---|---|
| PointCloud2输入 | HARDWARE_SINGLE_PASS | docs/B0_MAZE_SENSOR_CALIBRATION.md、静止 bags | base_link，历史约14.7Hz | B2.1-A当前配置频率待真机验收 |
| Odom输入 | HARDWARE_SINGLE_PASS | 同上 | odom->base_link，历史约149Hz | motion onset 与漂移未验收 |
| IMU输入 | HARDWARE_SINGLE_PASS | docs/B0_MAZE_SENSOR_CALIBRATION.md | 历史约250Hz | 本轮未复测 |
| 五扇区 | HARDWARE_SINGLE_PASS | docs/B1_MAZE_PERCEPTION_DRY_RUN.md | 基础标定 | 不能覆盖矩形转弯扫掠 |
| 后方三扇区 | BLOCKED | 静止 bags、maze_first_turn_core.py | rear coverage insufficient，fail-closed | 覆盖不足，后退禁用 |
| UDP运动桥 | IMPLEMENTED | src/rk_go2_sdk_bridge/、B1.5文档 | 源码存在 | 当前构建/SDK ACK未确认 |
| StopMove | HARDWARE_REPEAT_PASS | docs/MAZE_ACCEPTANCE_MATRIX.md | 历史0.25m/s 10/10记录 | 原始逐次日志未归档 |
| watchdog | HARDWARE_SINGLE_PASS | 验收矩阵历史记录 | 用户报告通过 | B2.1 Bool真实发布源未确认 |
| 遥控急停 | HARDWARE_SINGLE_PASS | 验收矩阵历史记录 | 用户报告通过 | 时间/距离原始证据未归档 |
| 状态机路线 | DRY_RUN_PASS | docs/B2_MAZE_NAVIGATION_POLICY_DRY_RUN.md | 曾推进route=1/5 | Round15物理碰撞 |
| Round15重放 | DRY_RUN_PASS | /tmp/claude_handoff_round15_replay_current.json | 接触前左侧风险 | 旧摘要版本不一致 |
| 0.413m粗保护 | DRY_RUN_PASS | B2文档、旧B2测试 | 约4.52deg拒绝旧路径 | 仅粗保护，不能替代完整几何 |
| 局部点云地图 | UNIT_TEST_PASS | tests/test_maze_first_turn_core.py | 纯逻辑通过 | 静止真机频率/过滤未验收 |
| 墙线/墙端 | UNIT_TEST_PASS | 同上、Round15当前重放 | 完整墙/审计短片段分层 | 多帧稳定未实现或未验收 |
| 动态矩形足迹 | UNIT_TEST_PASS | tests/test_maze_first_turn_core.py | 四向非对称+余量 | 余量未按冻结步态标定 |
| 连续轨迹检查 | UNIT_TEST_PASS | tests/test_maze_first_turn_core.py | vx/vy/wz、端点、原始点、尾程、扰动 | 无可执行候选 |
| 停止尾程模型 | IMPLEMENTED | maze_first_turn_core.py | 参数接口存在 | 真机停止尾程未标定 |
| Odom motion onset | UNVERIFIED | 未发现专用 MotionOnsetDetector | Yaw unwrap/跳变检查存在 | 静止Yaw漂移未解决 |
| 步态冻结 | NOT_STARTED | docs/MAZE_MASTER_PLAN.md | 当前灵动模式未冻结 | 未比较 StaticWalk/ClassicWalk/EconomicGait |
| 真实运动模型 | NOT_STARTED | config/maze_first_turn_dry_run.yaml | primitives均UNVALIDATED | 不能执行 |
| 短直行自动测试 | NOT_STARTED | 无 | 无 | B2.1-B未通过 |
| 短后退自动测试 | BLOCKED | rear coverage状态 | 后方盲区禁止 | 后向三扇区不足 |
| 第一弯单次 | FAIL | ROUND15 | 左侧轻触 | 新方案未获运动准入 |
| 第一弯连续3次 | NOT_STARTED | 无 | 无 | 单次未通过 |
| 单独右弯 | NOT_STARTED | 无 | 无 | 第一弯未验收 |
| 五弯完整路线 | NOT_STARTED | 无 | 无 | 不得跳过阶段 |
| 巡线入口交接 | NOT_STARTED | 无 | 无 | B3之后 |
| 巡线出口交接 | NOT_STARTED | 无 | 无 | B3之后 |

## 高优先级风险与禁止事项

1. 禁止非零运动。当前没有 EXECUTABLE_SAFE，也没有冻结步态、运动模型、停止尾程或安全 Bool Topic 的真机证据。
2. 禁止自动后退。rear、left_rear、right_rear 覆盖不足即为未知，不可解释成空旷。
3. 禁止以降低净空阈值修复锁止。静止入口 bag 出现自身前腿点云；只允许通过可审计的局部自身过滤辨别，同时保留正前方和侧墙障碍观测。
4. 禁止扩大整体机身过滤矩形。它会遮蔽正前方近障；仅可验证默认关闭的局部前腿过滤。
5. 禁止把 route=1/5、单测或回放 PASS 写成物理首弯 PASS。Round15物理结果是 FAIL。
6. 禁止启动运动桥、完整比赛 launch、gait_control_node真实执行模式，或直接向 /navigation/cmd_vel 发布非零 Twist。
7. 禁止删除 rosbag、日志、evidence/、build/install/log 或既有未提交修改。

## Claude 的唯一下一任务与验收

任务名：B2.1-A 静止真机感知与自身过滤验证。

允许读取、纯单测、离线 Round15 回放、静止传感器订阅、只读 B2.1 节点和 rosbag 录制。禁止任何非零运动或启动会连接 SportClient.Move 的进程。

先复现纯逻辑基线：

~~~bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m compileall scripts
~~~

真机静止验证必须保存 bag，且同时满足：

- 点云局部图稳定不低于约10Hz；
- map_statistics 可区分 height_filtered_points、body_filtered_points 和 leg_self_filtered_points；
- 正前方已知挡板与左右侧墙仍可见，局部前腿点簇被单独过滤；
- 不再出现由前腿自回波导致的 current_footprint_collision 或 current_hard_clearance_too_small；
- 后向三扇区覆盖结果被记录。若不足，明确保持后退禁用；
- motion_output=false、execution_allowed=false，无 Twist 或 SDK 发布器。

满足以上只是 B2.1-A 静止门槛，不是 B2.1-B 或任何移动准入。之后才是 B2.1-B步态冻结、运动模型和停止尾程标定；B2.1-C人工逐次ARM短直行5/5，后退仅在覆盖充分时5/5；B2.1-D第一弯分段执行；B2.1-E首弯连续3次零接触；随后依次为B2.2-A右弯、B2.2-B五弯、B3巡线交接、B4故障注入与封版。

每次工作结束，Claude 必须更新 config/maze_project_status.yaml、docs/MAZE_PROGRESS_TRACKER.md、docs/MAZE_ACCEPTANCE_MATRIX.md、docs/MAZE_TEST_EVIDENCE_INDEX.md，并输出Git状态、实际测试命令、是否发送非零命令、证据路径、PASS/FAIL/UNVERIFIED 与下一唯一任务。未获用户明确授权时不得提交或推送。
