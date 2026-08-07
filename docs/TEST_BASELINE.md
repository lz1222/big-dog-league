# 测试基线

## 结论边界

历史仓库卫生基线对应 `codex/repository-hygiene`、起点提交 `3c25a58`。本轮
控制权增量对应 `codex/national-control-architecture`、基线提交 `ded2631`。
VM 验证环境为 ROS 2 Humble、Python 3.10.12、pytest 6.2.5；机器人运行目标
仍是 ROS 2 Foxy。

VM 上的语法检查、pytest、colcon build/test 不能证明：

- Unitree Go2 实机能够连接或正确执行 SDK/UDP 命令；
- D1 机械臂收到命令、返回硬件 ACK 或完成真实抓放；
- 跳跃、台阶、避障、巡线和停靠在比赛场地通过；
- 相机输入、标志识别和平台选择在现场可靠；
- `/navigation/cmd_vel` 在机器人真实组合中只有一个 publisher，且没有误启
  standalone/tool 节点；
- VM/Humble 已验证的 SetBool/topic 急停软件闭环在机器人/Foxy 上具有相同
  DDS/时序行为，或能让机器人经 UDP/SDK 实体停止。

## 环境和测试分层

| 层级 | VM / Humble | 机器人 / Foxy | 本轮状态 |
| --- | --- | --- | --- |
| Git、Python、Shell、YAML、launch 静态检查 | 可执行 | 可复核 | preflight 可执行 |
| command mux 纯 Python 核心 | 可执行 | 可复核 | 最近定向运行 `36 passed` |
| command mux 隔离 ROS smoke | source Humble 后可执行 | source Foxy 后必须复核 | 最近定向运行 `1 passed` |
| 无硬件 Python 感知测试 | 可执行 | 可执行但非必要 | 14 项通过 |
| ROS Python bridge 测试 | source Humble 后可执行 | source Foxy 后应复核 | 2 项通过 |
| 各 ROS 包 pytest | source ROS 后逐包执行 | 可复核 | 74 项：65 passed、9 skipped、0 failed |
| `colcon build/test` | 临时目录干净构建 | 必须再次执行 | build/test 通过；54 tests、0 errors、0 failures、1 skipped |
| Go2 SDK/UDP、D1、相机和实物动作 | 不作为硬件验证 | 必须现场执行 | SKIP |

## 原始 22 个 pytest 收集错误

未 source ROS、未配置工作区 `PYTHONPATH` 时执行根级 `python3 -m pytest --collect-only -ra`，结果为 `3 collected / 22 errors`，退出码 2：

| 分类 | 数量 | 说明 |
| --- | ---: | --- |
| 重复测试模块名/文件命名冲突 | 18 | 7 个包重复使用 `test_copyright.py`、`test_flake8.py`、`test_pep257.py`；首包导入后，其余 6 包各产生 3 个 import mismatch。两个分类描述同一组错误，不能重复计数 |
| Python 包路径未配置 | 3 | 两个 perception 测试找不到 `rk_perception`，bridge 测试找不到 `rk_unitree_driver` |
| D1 动态库缺失 | 1 | `third_party/unitree_d1_sdk/src/test_grasp.py` 在导入阶段硬编码加载 `/home/unitree/d1_sdk/build/libleft_1.so` |
| ROS 未 source | 原始 22 中为 0 | 包路径错误先发生；修正 `PYTHONPATH` 后，bridge 会暴露缺少 `rclpy` |
| ament/ROS 依赖缺失 | source Humble 后为 0 | 当前 Humble 环境可导入 ament lint、rclpy 和 launch_testing |
| 原始收集中的真实断言失败 | 0 | 收集尚未进入测试执行 |

裸 `pytest` executable 当前不在 PATH，返回 127；`python3 -m pytest` 模块可用。

### D1 收集安全边界

`third_party/unitree_d1_sdk/src/test_grasp.py` 不只是普通测试：它在 import 阶段加载多个 vendor `.so`，并包含直接机械臂调用和等待。即使只做 pytest collection，只要库与硬件环境存在，也可能产生真实硬件副作用。

因此统一脚本始终：

- 将原始“全仓 pytest 收集”标记为 `SKIP`；
- 不根据 `.so` 是否存在自动运行 D1 测试；
- 只对 `src/` 做无硬件安全收集；
- 真实 D1 测试必须由人工安全流程单独执行。

## 仓库卫生历史安全测试结果

### 定向功能测试

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="src/rk_perception:src/rk_unitree_driver:${PYTHONPATH:-}" \
python3 -m pytest -q -p no:cacheprovider \
  src/rk_perception/test/test_real_line_tracker_node.py \
  src/rk_perception/test/test_real_sign_detector_node.py \
  src/rk_unitree_driver/test/test_cmd_vel_bridge_node.py
```

结果：`16 passed`，其中 perception 14 项、cmd_vel bridge 2 项。

### 安全收集与逐包 pytest

安全收集使用 `--import-mode=importlib`、关闭 pytest cache，并禁用会重新按传统方式导入同名测试模块的 launch testing collection hook；范围限定为 `src/`，不接触 D1 vendor 测试。

逐个 `src/*/test` 目录独立执行的结果为：

```text
37 tests total
28 passed
9 skipped
0 failed
```

此前唯一真实失败是 `rk_tools/test/test_pep257.py`：`obstacle_direct_route_node.py` 三个 docstring 使用中文句号，共产生 6 条 D400/D415。现已仅将这三个 docstring 的句末改为 PEP257 接受的 ASCII 句号，包作用域测试通过；未修改测试断言或忽略规则。

### colcon

统一脚本使用独立的 `/tmp/rk_workspace_test.*`：

```bash
colcon --log-base <tmp>/log build \
  --base-paths src \
  --symlink-install \
  --build-base <tmp>/build \
  --install-base <tmp>/install

colcon --log-base <tmp>/log test \
  --base-paths src \
  --build-base <tmp>/build \
  --install-base <tmp>/install

colcon test-result \
  --test-result-base <tmp>/build \
  --verbose
```

仓库卫生基线的干净 build 和 `colcon test` 均成功，当时结果为
`17 tests, 0 errors, 0 failures, 1 skipped`，全部来自 `rk_perception`。本轮
在 estop service 增量前曾于独立 `/tmp` 目录得到
`52 tests, 0 errors, 0 failures, 1 skipped`；新增 service/cache 语义测试后，
本轮重新干净构建并得到
`54 tests, 0 errors, 0 failures, 1 skipped`。任何 VM 结果都不代表机器人
Foxy 或硬件通过。

## 本轮 command mux 定向测试

纯核心测试不依赖 ROS graph 或真实硬件，覆盖优先级、超时、gait lock 不回退、
三种解锁清旧缓存、时间倒退、NaN/Inf 拒绝、clamp、输入对象不修改和 JSON
finite 状态：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="src/rk_safety:${PYTHONPATH:-}" \
python3 -m pytest -q -p no:cacheprovider \
  src/rk_safety/test/test_command_mux_core.py
```

最新纯核心定向记录：`36 passed`。新增覆盖 estop 置位和解除两种真实状态转换
清空命令/时间戳，以及重复 true/false 的幂等语义。

隔离 ROS smoke 使用唯一测试 topic 前缀，不连接 `/navigation/cmd_vel` 或真实
机器人，验证 mission candidate -> mux output、Bool topic estop、SetBool
service estop、两者共享转换语义、解除后旧命令不恢复和新命令恢复：

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="src/rk_safety:${PYTHONPATH:-}" \
python3 -m pytest -q -p no:cacheprovider \
  src/rk_safety/test/test_command_mux_node.py
```

最新隔离 ROS smoke 记录：`1 passed`。与纯核心合计
`src/rk_safety/test` 为 `37 passed`。测试接口全部使用随机隔离前缀，并额外
remap 默认最终话题，没有向真实 `/navigation/cmd_vel` 发布。

ROS smoke 只能证明测试 graph 上的软件消息路径。它没有覆盖：

- 真实 launch 中 `/navigation/cmd_vel` 的唯一 publisher；
- 机器人/Foxy 上 SetBool service 与 Bool topic 的 DDS/时延行为；
- `stop_line_system.sh` 的正常 service -> mux zero 确认和异常 fallback；
- UDP forwarder 的二次限幅、UDP server 或 Unitree SDK ACK；
- gait/arm lock 的真实生产者和硬件动作；
- 机器人 Foxy、相机、跳跃、台阶、机械臂或 170 分任务。

## 统一入口

```bash
bash scripts/national_preflight.sh
bash scripts/test_workspace.sh
```

`test_workspace.sh`：

- 使用 `set -Eeuo pipefail`，但每个步骤捕获返回码后继续；
- 输出 `PASS/WARN/FAIL/SKIP`；
- 自动沿用已 source 的 ROS，或按架构尝试 source Humble/Foxy；
- 把 Python cache、colcon build/install/log 和测试日志写入新建的 `/tmp`；
- 逐包运行安全 pytest，并保留真实失败；
- 默认清理自己创建的临时目录，可用 `RK_KEEP_TEST_ARTIFACTS=1` 保留日志；
- 比较运行前后的 `git status`，确认测试没有向工作区写入 Git 可见文件；
- 硬件、网络、相机和 D1 项明确 SKIP。

estop service 增量落盘后的最终汇总：

- `national_preflight.sh`：`PASS=21 WARN=7 FAIL=0`；
- `test_workspace.sh`：`PASS=21 WARN=1 FAIL=0 SKIP=4`；
- 安全 `src/` 收集：74 项；逐包执行为 65 passed、9 skipped、0 failed；
- 干净 colcon：build/test 通过，`54 tests, 0 errors, 0 failures, 1 skipped`。

危险 D1 全仓收集及 Go2/Foxy、D1、相机/实物硬件项仍保持
SKIP/WARN，不能因软件测试通过而改写为成功。

## 当前不可在 VM 完成

- Foxy 机器人端全量构建和运行时 topic/action 检查；
- `/navigation/cmd_vel` 唯一发布者和 command mux 超时/lock/恢复的机器人端验证；
- SetBool service、Bool topic、mux 零速和断线/fallback 的机器人端验证；
- Unitree SDK/UDP server 返回、运动和急停验证；
- D1 vendor 动态库、真实 subscriber、命令 ACK 和负载动作；
- D435i 真实帧、场地识别与完整比赛路线；
- 跳跃、台阶、避障、机械臂和停靠硬件验收。

因此即使 command mux 核心、隔离 ROS smoke、preflight、workspace tests 和 colcon
全部通过，也只代表控制权软件基础在相应测试范围内通过；不能把它写成跳跃、
台阶、机械臂、完整比赛、170 分或硬件验证完成。
