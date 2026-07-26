# 测试基线

## 结论边界

本基线对应 `codex/repository-hygiene`，起点提交为 `3c25a58`。VM 验证环境为 ROS 2 Humble、Python 3.10.12、pytest 6.2.5；机器人运行目标仍是 ROS 2 Foxy。

VM 上的语法检查、pytest、colcon build/test 不能证明：

- Unitree Go2 实机能够连接或正确执行 SDK/UDP 命令；
- D1 机械臂收到命令、返回硬件 ACK 或完成真实抓放；
- 跳跃、台阶、避障、巡线和停靠在比赛场地通过；
- 相机输入、标志识别和平台选择在现场可靠；
- `/navigation/cmd_vel` 在完整硬件组合中只有一个发布者。

## 环境和测试分层

| 层级 | VM / Humble | 机器人 / Foxy | 本轮状态 |
| --- | --- | --- | --- |
| Git、Python、Shell、YAML、launch 静态检查 | 可执行 | 可复核 | preflight 可执行 |
| 无硬件 Python 感知测试 | 可执行 | 可执行但非必要 | 14 项通过 |
| ROS Python bridge 测试 | source Humble 后可执行 | source Foxy 后应复核 | 2 项通过 |
| 各 ROS 包 pytest | source ROS 后逐包执行 | 可复核 | 27 通过、9 跳过、1 失败 |
| `colcon build/test` | 临时目录干净构建 | 必须再次执行 | VM 构建和已注册测试通过 |
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

## 当前安全测试结果

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
27 passed
9 skipped
1 failed
```

唯一真实失败是 `rk_tools/test/test_pep257.py`。`obstacle_direct_route_node.py` 三个 docstring 使用中文句号，产生 6 条 D400/D415。根据本轮禁改 `src` 的约束，该问题只记录，不修改源码、断言或 skip 规则。

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

干净 build 和 `colcon test` 均成功。当前 colcon 结果为 `17 tests, 0 errors, 0 failures, 1 skipped`，全部来自 `rk_perception`；其他包虽然有 `test/`，但没有全部注册到 colcon 测试流程。因此这个结果只证明已注册的 perception 测试通过，不代表全仓 pytest 或硬件通过。

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

本轮统一脚本实际结果：

```text
PASS=19 WARN=1 FAIL=1 SKIP=4
```

唯一 FAIL 是上述 `rk_tools` pep257；WARN 是当前审计分支本来就有索引清理和文档变更。四个 SKIP 为危险的全仓 D1 收集，以及 Go2/Foxy、D1、相机/实物硬件验证。

## 当前不可在 VM 完成

- Foxy 机器人端全量构建和运行时 topic/action 检查；
- Unitree SDK/UDP server 返回、运动和急停验证；
- D1 vendor 动态库、真实 subscriber、命令 ACK 和负载动作；
- D435i 真实帧、场地识别与完整比赛路线；
- 跳跃、台阶、避障、机械臂和停靠硬件验收。
