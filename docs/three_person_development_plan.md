# 三人并行开发总纲

本文档把赛规、评分点、3D 地图和当前 ROS2 `src` 状态整理成三人可并行推进的分工总纲。三人只通过冻结 topic/action 交互，不互相改对方核心节点。

## 赛规约束

| 约束 | 对开发的影响 |
| --- | --- |
| 上场时间 10 分钟 | 启动、调试、两次机会都包含在 10 分钟内，必须准备一键启动、快速 stop、分段调试流程。 |
| 两次机会取最好成绩 | 参数和动作要稳定可重复，第一轮保守拿分，第二轮再尝试高风险动作。 |
| 人工干预限时 10 秒，扣 30 分 | 所有异常优先自动停车，避免失控导致人工干预。 |
| 黑色导引线宽约 100mm | 巡线视觉参数以 100mm 黑线为核心目标，需兼容光照和磨损。 |
| 场地存在尺寸、角度、拼接、颜色、磨损误差 | 不依赖单一固定时长完成全程，关键区要有视觉/状态校验和失败兜底。 |

## 冻结接口

本轮文档和后续代码计划都不得修改现有 topic/action/msg 字段。

| 类型 | 名称 | 用途 | 主要负责人 |
| --- | --- | --- | --- |
| Topic | `/perception/line_track` | 视觉巡线结果 | 成员 1 |
| Topic | `/navigation/cmd_vel` | 导航速度输出 | 成员 1/2 |
| Topic | `/gait/control_lock` | 固定动作占用控制权时暂停导航速度 | 成员 2 |
| Topic | `/mission/start` | 状态机启动巡线 | 成员 3 |
| Topic | `/mission/stop` | 状态机停止巡线 | 成员 3 |
| Action | `/mission/run` | 完整任务入口 | 成员 3 |
| Action | `/locomotion/execute_motion` | 固定运动动作入口 | 成员 2/3 |
| Action | `/arm/execute_task` | 机械臂动作入口 | 成员 3 |

## 三人职责

| 成员 | 主职责 | 主负责文件 | 不做的事 |
| --- | --- | --- | --- |
| 成员 1 | 巡线视觉 + 导航 | `src/rk_perception/rk_perception/real_line_tracker_node.py`, `src/rk_navigation/rk_navigation/line_follower_node.py`, `src/rk_perception/config/perception.yaml`, `src/rk_navigation/config/navigation.yaml`, `src/rk_bringup/config/line_nav_params.yaml`, `src/rk_bringup/launch/vision_nav_debug.launch.py`, `src/rk_bringup/launch/competition_line_nav.launch.py` | 不写机械臂动作，不直接调 Go2 底层，不决定完整比赛流程。 |
| 成员 2 | 步态运动 + Unitree 控制 + 避障区通过 | `src/rk_locomotion/rk_locomotion/gait_control_node.py`, `src/rk_locomotion/config/gait_params.yaml`, `src/rk_unitree_driver/rk_unitree_driver/cmd_vel_bridge_node.py`, `src/rk_unitree_driver/rk_unitree_driver/go2_motion_client.py`, `src/rk_unitree_driver/rk_unitree_driver/safety_monitor.py`, `src/rk_go2_sdk_bridge/scripts/cmd_vel_udp_forwarder.py`, `src/rk_tools/rk_tools/two_step_walk_test_node.py` | 不改视觉识别算法，不私自改 `/navigation/cmd_vel` 名称，不绕过 stop/watchdog。 |
| 成员 3 | 任务状态机 + 机械臂 + 识别映射 | `src/rk_mission/rk_mission/mission_state_machine_node.py`, `src/rk_config/config/mission/competition.yaml`, `src/rk_config/config/arm/d1_presets.yaml`, `src/rk_tools/rk_tools/mock_arm_server.py`, `src/rk_tools/rk_tools/mock_locomotion_server.py`, `src/rk_tools/rk_tools/mission_client_node.py`, `src/rk_interfaces/*` | 不重写底层步态，不在状态机里直接发 Go2 底层速度，不随意改接口字段。 |

## 得分点责任映射

| 得分点 | 分值 | 主负责人 | 输入 | 输出 | 验收标准 |
| --- | --- | --- | --- | --- | --- |
| 起点障碍 | 10 | 成员 2 | 成员 3 调用 `/locomotion/execute_motion` 的 `start_jump`；成员 1 提供跳障前停车 | Go2 跳障动作、落地 stop/hold | 全程不碰起点障碍，落地后姿态稳定。 |
| 避障区域 | 20 | 成员 2 | 成员 3 停止巡线并调用避障动作；成员 1 提供入口、出口、黑线恢复判断 | 低速固定动作序列、`/gait/control_lock`、出口恢复巡线 | 从入口进入、出口离开，尽量不碰挡板；碰挡板次数可记录。 |
| 台阶区域 | 20/30 | 成员 2 | 成员 3 调用 `stairs_up_down`；成员 1 提供台阶前停车/可选 Aruco 检测 | 上下台阶动作、恢复站姿 | 不从台阶掉落；至少每个足端接触第二级顶面，争取最高级顶面。 |
| 抓取平台 | 10 | 成员 3 | 成员 1 导航到平台前；物资标签/平台外观；机械臂预设 | `/arm/execute_task` 抓取起始物资 | 起始物资离开抓取平台顶面。 |
| 中转平台卸载 | 20 | 成员 3 | 已抓起起始物资；平台前定位 | `/arm/execute_task` 放置起始物资 | 起始物资稳定停留在中转平台顶面。 |
| 中转平台抓取 | 10 | 成员 3 | 场地物资标签/平台外观；机械臂预设 | `/arm/execute_task` 抓取场地物资 | 场地物资离开中转平台顶面。 |
| 检测平台 | 20 | 成员 3 | 成员 1 停在检测点；警示标志识别 | 警示动作：伸懒腰、打招呼或前灯闪三次 | 机器人垂直投影覆盖检测点，并执行正确动作。 |
| 放置平台 | 30 | 成员 3 | 抓取平台 1/2 标志识别结果；场地物资已抓起 | `/arm/execute_task` 放到对应平台 | 场地物资稳定停留在对应一号/二号放置平台顶面。 |
| 终点障碍 | 10 | 成员 2 | 成员 3 调用 `finish_jump`；成员 1 提供障碍前停车 | 终点跳障动作、落地 stop/hold | 全程不碰终点障碍。 |
| 终点停靠 | 10 | 成员 1/3 | 成员 3 进入返回启停区阶段；成员 1 巡线停靠 | `/mission/stop`、零 `/navigation/cmd_vel` | 四个足端完全位于启停区蓝色区域内并稳定停止。 |
| 报告材料 | 30 | 全员 | 三人测试记录、视频、问题分析 | 报告材料 | 至少包含作品概述、比赛程序、问题分析、技术方案与结果感想。 |
| 人工干预 | -30 | 全员 | 失控、卡死、撞障碍、超时风险 | stop 脚本、遥控器、安全策略 | 尽量不触发人工干预；若必须干预，10 秒内完成且不改变有效距离。 |

## 避障区专项分工

避障区采用保守方案：状态机切换 + 低速固定动作序列 + 出口恢复巡线。

| 成员 | 责任 | 输入 | 输出 | 验收标准 |
| --- | --- | --- | --- | --- |
| 成员 1 | 判断避障区入口、出口、黑线恢复 | `/perception/line_track`、障碍区入口/出口外观、黑线连续性 | `line_visible` 恢复条件、入口/出口调参记录 | 入口前能停车，出口后能重新稳定跟线。 |
| 成员 2 | 低速进入、转向、通过、退出、STOP 保护 | `/locomotion/execute_motion` 或后续 JSON 动作映射、Go2 状态 | `ENTER_OBSTACLE_ZONE`、`OBSTACLE_FORWARD_SLOW`、`OBSTACLE_TURN_LEFT`、`OBSTACLE_TURN_RIGHT`、`OBSTACLE_SIDE_ADJUST`、`EXIT_OBSTACLE_ZONE`、`OBSTACLE_STOP` | 动作期间 `/gait/control_lock=true`，速度保守，任意失败能 stop。 |
| 成员 3 | 状态机切换：巡线停止 -> 调用避障动作 -> 恢复巡线 | `/mission/start`、`/mission/stop`、`/locomotion/execute_motion` | `LINE_TO_OBSTACLE_ENTRY` 到 `RESUME_LINE_FOLLOWING` 的状态转换 | 巡线和固定动作不抢 `/navigation/cmd_vel`，失败进入 stop 或重试一次。 |

## 当前代码缺口与优先级

| 优先级 | 缺口 | 负责人 | 处理要求 |
| --- | --- | --- | --- |
| P0 | `line_follower_node.py` 合并冲突标记必须检查并修复 | 成员 1 | 合并前执行 `rg -n "<<<<<<<|=======|>>>>>>>" src`，若命中必须先修复；即使当前已清零，也作为 P0 验收项保留。 |
| P0 | 真实 `/locomotion/execute_motion` action server 缺失 | 成员 2 | `rk_mission` 已调用该 action，但真实 `gait_control_node` 目前主要是 `/gait/command_json`；后续要补 action server 并复用现有动作逻辑。 |
| P0 | 完整比赛 launch 尚未接入 mission/gait/arm | 成员 3 | 规划 `competition_full.launch.py`，集成 RealSense、真实巡线、导航、gait、cmd_vel bridge、mission、arm mock/adapter。 |
| P1 | `rk_config/config/arm/d1_presets.yaml` 机械臂预设为空 | 成员 3 | 先定义预设格式和 mock 读取方式，再接真实机械臂 SDK。 |
| P2 | `rk_common` 目前为空骨架 | 全员 | 暂不塞核心逻辑；只有出现跨包重复工具且接口稳定后再抽取。 |

## 分支与合并规则

| 成员 | 分支名 | 合并前必须完成 |
| --- | --- | --- |
| 成员 1 | `feature/vision-nav` | 巡线和导航单测通过；无 conflict 标记；关键路线段有调参记录。 |
| 成员 2 | `feature/locomotion-go2` | Go2 低速、转向、stop、watchdog、避障固定动作可单独验证。 |
| 成员 3 | `feature/mission-arm` | mock 全流程可跑；机械臂预设格式明确；状态机支持分段执行。 |

## 必写测试命令

```bash
rg -n "<<<<<<<|=======|>>>>>>>" src
python3 -m py_compile src/rk_navigation/rk_navigation/line_follower_node.py
python3 -m py_compile src/rk_mission/rk_mission/mission_state_machine_node.py
python3 -m py_compile src/rk_locomotion/rk_locomotion/gait_control_node.py
PYTHONPATH=src/rk_perception:$PYTHONPATH python3 -m pytest src/rk_perception/test/test_real_line_tracker_node.py
colcon build --symlink-install
```

## 阶段推进

| 阶段 | 目标 | 成员 1 | 成员 2 | 成员 3 |
| --- | --- | --- | --- | --- |
| 第 1 阶段 | 基础链路 | 真实/Mock `LineTrack` 到 `cmd_vel` | `cmd_vel` 到 Go2，stop 可用 | `/mission/start` 和 `/mission/stop` 可控 |
| 第 2 阶段 | 障碍和台阶 | 到入口/台阶前停车与恢复巡线 | 起终点跳障、避障、台阶动作库 | 状态机切换巡线和固定动作 |
| 第 3 阶段 | 机械臂和识别 | 平台前定位 | 平台前低速微调 | 抓放、标志映射、警示动作 |
| 第 4 阶段 | 全流程 | 盯视觉 debug 和丢线 | 盯姿态、安全和动作稳定性 | 盯状态机日志、超时和得分路径 |

