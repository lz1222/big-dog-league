# 非机械臂比赛软件验收

本文对应 `competition_non_arm.launch.py`。它只覆盖巡线、起终点白横线
FrontJump、红圈后的警示牌动作与终点停车的软件链路；不是机械臂、避障或楼梯
功能的完成声明。

## 范围与明确排除项

已纳入的软件链：D435i 图像、真实巡线和警示牌检测、`line_follower_node`、
`line_course_mission_node`、白横线 stage/executor、真实
`gait_control_node` Action server、inspection executor、`command_mux_node` 和
硬件模式下的 SDK UDP 后端。

明确排除：机械臂/抓取/中转/放置，避障区/迷宫/避障路线，楼梯区及
`stairs` 动作，以及这些项目的任何实机功能。也不把 VM 软件测试表示为
FrontJump、三种 SDK 姿态、UDP 传输或实体停止已经在机器人上验证。

## 正式启动、readiness 与任务控制

在机器人 Foxy 环境中完成构建并 source 后，启动（不自动起跑）：

```bash
cd ~/桌面/rk_inspection_ws
./src/rk_bringup/scripts/start_non_arm_competition.sh
```

`mission_start.sh`、`mission_stop.sh` 和启动/停止入口会从自身的源码或
`install/rk_bringup/share/rk_bringup/scripts` 路径推导工作区；只有使用非标准
部署位置时才需要显式设置 `RK_INSPECTION_WS`。

启动脚本建立独立 `rk_non_arm_competition` tmux session，并在以下目录建立
camera、tracker、sign detector、line follower、line course、white stage、white
executor、inspection executor、gait、mux、UDP forwarder、SDK server 的日志入口：

```bash
${RK_COMPETITION_LOG_DIR:-$HOME/rk_non_arm_competition_logs}
```

只读 readiness 命令：

```bash
ros2 service call /competition/check_readiness std_srvs/srv/Trigger '{}'
ros2 topic echo /competition/readiness_status
```

`mission_start.sh` 在 readiness 成功后只发布一次 `/mission/start`，并确认
`mission_started=true`、非空 `run_id` 和 `route_phase=START_STAGE`：

```bash
./src/rk_bringup/scripts/mission_start.sh
```

停止顺序固定为 mission stop、等待白横线与 inspection action 终止、mux estop、
连续最终零速度、最后停止进程：

```bash
./src/rk_bringup/scripts/mission_stop.sh
./src/rk_bringup/scripts/stop_line_system.sh
```

正常路径不直接向 `/navigation/cmd_vel` 发布 Twist。只有 stop 脚本明确打印
`EMERGENCY FALLBACK` 时才会单次直发零 Twist；该分支不计入验收通过。

## 预期 ROS 图与控制权

```text
D435i -> real_line_tracker_node -> line_follower_node
      -> line_course_mission_node -> /control/mission_cmd
      -> command_mux_node -> /navigation/cmd_vel
      -> cmd_vel_udp_forwarder -> go2_sdk_udp_server

white_bar_stage_command_publisher -> /mission/white_bar_stage_command
      -> line_course_mission_node -> white_bar_action_executor
      -> /locomotion/execute_motion -> gait_control_node -> FrontJump helper

real_sign_detector_node -> /perception/sign_detections
      -> inspection_action_executor -> SDK helper
      -> matching request_id status -> line_course_mission_node
```

`command_mux_node` 是 `/navigation/cmd_vel` 的唯一正式发布者。启动或验收时应
核对：

```bash
ros2 topic info -v /navigation/cmd_vel
```

预期 publisher count 为 `1`，节点名为 `command_mux_node`；forwarder 仅是订阅者。

主要话题：`/perception/line_track`、`/navigation/line_follow_status`、
`/mission/line_course_state`、`/mission/white_bar_stage_command`、
`/mission/white_bar_action_status`、`/mission/inspection_action_status`、
`/safety/estop_state`、`/control/cmd_mux_status`。唯一 motion Action 是
`/locomotion/execute_motion`。

白横线 timeout 链在统一配置中为：起点/终点 FrontJump 最坏软件时长 `17s`，
executor `22s`，line-course `26s`；自动测试要求每层比下一层至少多 `3s`。

## 软件 smoke

在 VM/Humble 上使用以下命令：

```bash
./scripts/accept_non_arm_competition.sh
```

脚本会使用独立 `ROS_DOMAIN_ID`，临时编译
`src/rk_bringup/test_support/fake_sdk_motion_helper.c` 为 ELF helper，并传入
`software_smoke_mode=true`。这个 helper 不调用 Unitree SDK、不打开网络接口，
并带有固定的 test-only marker；readiness、FrontJump 和 inspection executor 都会
拒绝未带 marker 的 ELF、`/usr/bin/true`、脚本或真实 SDK helper。脚本会在每次
验收目录下 build/install 独立 overlay，并把它显式 source 到 launch 和任务控制
脚本中，避免误测陈旧的工作区 `install`。软件 smoke 不启动 RealSense、UDP server
或 UDP forwarder；合成消息只驱动真实 line follower、line course、white executor、
gait Action server、inspection executor 和 command mux 的 ROS 内部流程。

验收期间会持续读取 `/proc`：任何 `go2_sdk_udp_server` 或真实
`go2_sdk_motion_action` 进程都会失败；每次 fake helper 调用还必须能以
`/proc/<pid>/exe` 精确匹配临时 ELF。默认总 timeout 为 900 秒，覆盖隔离 build 与
完整路线。可设置 `RK_KEEP_NON_ARM_ACCEPT_DIR=true` 保留日志，或仅在调用方已经
准备好同样隔离的 overlay 时设置 `RK_ROS_OVERLAY_SETUP=/abs/path/install/setup.bash`。

脚本带总 timeout，至少验证初始 `WAIT_START`、readiness、一次 start 与 run_id
保持、START/FINISH 两次真实 Action 流程、electric_shock -> stretch 的 inspection
闭环、最终 `FINAL_STOP` 和唯一最终速度发布者。单元测试覆盖三种精确映射和
fail-closed 的拒绝/超时/取消分支。

## Foxy / Python 3.8 测试兼容性复核

非机械臂正式链及其测试禁止在模块导入时依赖 Python 3.9+ 的内置泛型或
Python 3.10+ 的 `X | None` 标注。`test_non_arm_competition_contract.py` 会扫描
正式链的运行时标注，并实际导入核心模块；该检查不能被 Humble 的
`compileall` 替代。

`PersistentCleanupGuard` 的生产约束没有因测试移植而放宽：最终父目录必须为
当前用户的 `0700`，guard 必须为普通文件且权限为 `0600`，符号链接和不安全的
既有目录仍会 fail-closed。测试在 pytest 临时目录下另建并显式验证 `0700` 的
专用子目录；它不修改 pytest 提供的父目录。gait 测试关闭顺序为停止 worker、
显式销毁 ActionServer、销毁 Node、最后关闭 Context，以避免 Foxy 在析构阶段
访问已失效 handle。

在机器人 Foxy 上，先完成下面的软件测试（不启动 launch、不发送动作）后再进行
后续分项验收：

```bash
cd ~/桌面/rk_inspection_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  rk_interfaces rk_perception rk_navigation rk_mission rk_locomotion \
  rk_safety rk_go2_sdk_bridge rk_bringup
source install/setup.bash
colcon test --packages-select \
  rk_interfaces rk_perception rk_navigation rk_mission rk_locomotion \
  rk_safety rk_go2_sdk_bridge rk_bringup
colcon test-result --verbose
python3 -m compileall -q \
  src/rk_bringup src/rk_mission src/rk_navigation src/rk_locomotion src/rk_safety
python3.8 - <<'PY'
import rk_bringup.non_arm_competition_contract
import rk_navigation.start_ready_core
import rk_mission.inspection_action_core
import rk_mission.non_arm_route_phase_core
import rk_mission.white_bar_action_core
import rk_mission.white_bar_stage_command_core
import rk_locomotion.front_jump_supervisor
print('FOXY_PYTHON38_CORE_IMPORTS=PASS')
PY
```

## 机器人 Foxy 构建与分项实机验收

在机器人上同步 VM 源码后执行：

```bash
cd ~/桌面/rk_inspection_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  rk_interfaces rk_perception rk_navigation rk_mission rk_locomotion \
  rk_safety rk_go2_sdk_bridge rk_bringup
source install/setup.bash
./src/rk_bringup/scripts/start_non_arm_competition.sh
ros2 service call /competition/check_readiness std_srvs/srv/Trigger '{}'
ros2 topic info -v /navigation/cmd_vel
```

下列项目为 `NEEDS_HARDWARE_EVIDENCE`，必须在隔离、有人值守、可急停的机器人
场地分项记录，不能由 VM 结果替代：

- D435i 实际话题、帧率和现场视觉阈值；
- UDP server/forwarder 的真实接收、限幅与断链行为；
- START 与 FINISH FrontJump 的实体跨越、零速清理和恢复找线；
- `stretch`、`hello`、`blink_front_light_3` 三种 SDK 姿态的实体效果；
- mission stop、estop 和进程停止时机器人实际持续静止；
- 红圈、警示牌、白横线、终点蓝区在赛场光照下的阶段门控。

机械臂、避障区、楼梯区仍是明确排除项，不能在本启动入口中作为已验收功能报告。
