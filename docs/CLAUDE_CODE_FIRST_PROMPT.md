# Claude Code 首次接管提示词

你现在正式接手 Unitree Go2 窄迷宫自主避障项目。先完整阅读 docs/CLAUDE_CODE_MAZE_HANDOFF.md，再阅读：

~~~text
docs/MAZE_MASTER_PLAN.md
docs/MAZE_PROGRESS_TRACKER.md
docs/MAZE_ACCEPTANCE_MATRIX.md
docs/MAZE_TEST_EVIDENCE_INDEX.md
config/maze_project_status.yaml
docs/B2_1_FIRST_TURN_AUTONOMOUS_DRY_RUN.md
~~~

严格以这些文件和本机证据为准，不依赖旧聊天。

## 身份、任务与安全边界

项目目标是在人工监护下让 Go2 按 LEFT、LEFT、RIGHT、RIGHT、LEFT 通过约57cm最窄净宽迷宫，零接触。正式执行必须是人工逐次确认的短段：短段 -> StopMove -> 实体稳定 -> 重扫 -> 重规划。禁止原地旋转、固定时间猜角、无后方感知自动后退，以及直接绕过 command mux。

当前正式阶段是 B2.1-A，且 motion_allowed=false、first_turn_execution_allowed=false。当前唯一任务是：

~~~text
B2.1-A 静止真机感知与自身过滤验证
~~~

不得同时开始 B2.1-B、短直行、短后退、第一弯执行、第二弯、完整迷宫或巡线交接。

未经用户当轮明确授权，严禁：发布非零 Twist；调用 SportClient.Move、/api/sport/request 或运动桥；启动 go2_sdk_udp_bridge.launch.py、真实 gait_control_node、完整比赛 launch；自动ARM或切步态；直接向 /navigation/cmd_vel 发布非零速度；降低净空门限；扩大整体机身过滤矩形；删除 evidence/、rosbag、日志、build/install/log 或既有未提交修改；reset、clean、restore、自动commit或push。

如果一条命令可能实体运动，不运行，只标记 RISK_REQUIRES_HUMAN_REVIEW。

## 仓库、环境与 Git

预期Go2仓库是 /home/unitree/big-dog-league，但先验证：

~~~bash
pwd
git status --short --branch
git branch --show-current
git rev-parse HEAD
git log -10 --oneline
git diff --stat
git diff --cached --stat
git remote -v
~~~

交接审计工作区为 /home/lqsdaba/daihao1/big-dog-league；两处可能不同。最终审计快照是 master，HEAD 与 origin/master 均为 c220f11a92d3eba7be479ab667e7044f4f30edb7。审计期间 HEAD 从 16251ae 变为 c220f11，本轮未执行 Git 写操作；最终仅有本轮交接文档未跟踪。先报告差异，再讨论同步或合并。

Go2只读ROS2环境顺序：

~~~bash
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
~~~

不要在B2.1-A进程中加载 Unitree SDK2/CycloneDDS 运动进程。Python版本必须实测记录。

## 传感器、控制链与事实状态

输入：/utlidar/cloud_base 为 PointCloud2、frame=base_link、历史约14.7Hz；/utlidar/robot_odom 为 Odometry、odom->base_link、历史约149Hz；/utlidar/imu 为 Imu、历史约250Hz。

唯一正式运动链是 /control/locomotion_cmd -> command_mux_node -> /navigation/cmd_vel -> cmd_vel_udp_forwarder.py -> UDP -> go2_sdk_udp_server -> SportClient。command_mux_node必须是 /navigation/cmd_vel 的唯一正式Publisher。

scripts/maze_first_turn_dry_run.py 不是运动层：只读点云/Odom/watchdog/estop Bool，唯一业务输出为 /maze/first_turn/dry_run_status 的String JSON，并固定 motion_output=false、execution_allowed=false。

B2.1-A已有局部二维图、八扇区、后向fail-closed、有限墙段/端点、四向动态足迹、连续vx/vy/wz采样、停止尾程、扰动、六级候选排名。所有运动primitive仍为UNVALIDATED，不能执行。

rear/left_rear/right_rear当前覆盖不足时必须保持UNKNOWN和后退禁用。当前 HEAD 中的front_leg_self_filter_enabled默认关闭，仅用于静止验证，必须保留机头正前方和侧墙障碍观测。

## Round15

物理Round15是FAIL：第一左弯约44deg时左侧机身轻触内侧板。route=1/5只是旧策略状态机通过，不能写成物理通过。数据库为 evidence/round15/b2_round15_20260802_144706/b2_round15_20260802_144706_0.db3，SHA256为6788e8687343471cfae44d008a316ad271b2677ce233e5c303103e85fa7b3b19。

旧B2以sqrt((0.70/2)^2+(0.31/2)^2)+0.03约0.413m做粗扫掠保护；历史重放约4.52deg触发turn_sweep_unsafe，是DRY_RUN_PASS。

当前工作区只读重放产生 /tmp/claude_handoff_round15_replay_current.json：gate_status=DRY_RUN_PASS、physical_contact_result=FAIL、motion_output=false、publisher_count=0；匹配左侧风险在预测约23.980deg、接触前约7.278秒。全局最小预测净空仍0m、无ROBUST_SAFE候选、后向覆盖不足。旧geometry_summary*.json写为FAIL，和当前 HEAD 短片段逻辑不同；必须保留并记录复现版本、配置和SHA，不得选择性忽略。

## 本轮允许工作与验收

先运行纯逻辑基线：

~~~bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m compileall scripts
~~~

允许只读Round15重放、静止传感器订阅、只读B2.1节点和rosbag录制。静止验收目标：局部图约不低于10Hz；记录height_filtered_points、body_filtered_points、leg_self_filtered_points；正前挡板和两侧墙仍可见；前腿自回波不再造成当前足迹伪碰撞；记录后方三扇区覆盖，若不足继续禁用后退；全程保持motion_output=false、execution_allowed=false且无Twist/SDK发布器。

静止验收完成前不能进入B2.1-B。B2.1-B需要冻结并比较步态、标定运动模型、停止尾程和动态余量；B2.1-C才是人工逐次ARM的短直行5/5，后退仅在后方覆盖充分时5/5；之后依次B2.1-D单弯分段、B2.1-E单弯三连、B2.2-A右弯三连、B2.2-B五弯三连、B3巡线交接、B4故障注入。

## 每次工作结束必须输出

1. 实际仓库路径、分支、HEAD、git status --short --branch。
2. 修改文件及是否存在本轮前遗留修改。
3. 运行的命令和测试结果。
4. 是否发送过非零命令；未授权时必须为否。
5. rosbag/日志/视频证据路径和SHA。
6. 所有结论只能使用 UNIT_TEST_PASS、DRY_RUN_PASS、HARDWARE_SINGLE_PASS、HARDWARE_REPEAT_PASS、FAIL、BLOCKED 或 UNVERIFIED。
7. 更新 maze_project_status.yaml、进度、验收矩阵、证据索引后的差异。
8. 唯一下一任务及准入条件。

没有明确的人类许可，不要提交、推送、合并远端或执行实体运动。
