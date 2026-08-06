# d1_gripper_tx_probe — EXPERIMENTAL HARDWARE WRITER

此目录不属于 `rk_arm` 正式驱动、比赛 launch 或 ROS 图。它是为未来单次夹爪
验证准备的隔离实验程序；默认 CMake 配置会失败，必须明确开启。它始终保持
`manual_motion_enabled=false`、`STOP_SCHEMA_UNCONFIRMED`。

```bash
cmake -S tools/d1_gripper_tx_probe -B /tmp/d1_gripper_tx_probe_build \
  -DBUILD_D1_EXPERIMENTAL_WRITER=ON -DBUILD_TESTING=ON
cmake --build /tmp/d1_gripper_tx_probe_build -j2
(cd /tmp/d1_gripper_tx_probe_build && ctest --output-on-failure)
```

它只链接 Unitree SDK2/CycloneDDS，不链接 ROS、rclcpp 或 FastDDS。仅接受 `--seq`、
`--delta` 和网卡参数；没有关节编号、前六路目标、轨迹、循环、停止、使能或失能入口。
`delta` 在代码中硬限制为 `[-1.0, 1.0] app_display_unit`；`angle0` 至 `angle5`
从实时完整反馈逐值复制，只有 `angle6 = current_angle6 + delta`。

## 纯 dry run

```bash
env -u LD_LIBRARY_PATH /tmp/d1_gripper_tx_probe_build/d1_gripper_tx_probe \
  --interface eth0 --seq 60504 --delta 1.0
```

该命令只建立 `rt/arm_Command`、`rt/arm_Feedback` 与 `current_servo_angle` reader。
它先连续五秒确认 command topic 静默，再确认双反馈源有效、0.3 秒内新鲜、一致、有限，
且状态为 `(enable,power,error)=(1,0,0)`。随后完整 JSON 只序列化一次并冻结在当前进程
内存中，输出 JSON、SHA-256、快照 TTL 和 `DRY_RUN_ONLY / NOT SENT`；不创建本地锁或 DDS writer。

快照 TTL 固定为 15 秒，且不提供延长参数。快照绝不写入磁盘，也不能在另一进程恢复。

## 未来受控会话（本轮禁止执行）

未来仅在现场条件、官方急停手段和操作者明确授权均已满足时，才允许运行：

```bash
env -u LD_LIBRARY_PATH /tmp/d1_gripper_tx_probe_build/d1_gripper_tx_probe \
  --interface eth0 --seq 60504 --delta 1.0 --guarded-session --hardware-send
```

同一进程输出冻结 JSON 与 SHA 后，必须在 15 秒内从该程序标准输入输入它刚刚打印的：

```text
SEND_ONE_GRIPPER_TARGET <SHA256>
```

等待期间持续监听所有 reader。反馈过期、双源不一致、状态变化、任一通道相对冻结反馈
超过 `0.2 app_display_unit` 静止噪声容差、收到任意 command 帧或发现活跃本地锁，都会使
快照失效且不创建 writer。确认前绝不重新读取数据覆盖快照，也绝不重新编码 JSON。

仅当所有复核成立时，程序才在最小作用域内获取一次本地锁，断言 preview 与 Write 的字节、
长度和 SHA-256 一致，并使用该冻结字符串调用一次 `Write` 后立即关闭 writer。没有重试、
保持、自动回位、Stop、Enable、Disable 或第二帧。

`address=1`、`mode=0` 与 `funcode=2` 都是 **OBSERVED FROM OFFICIAL APP TRAFFIC**，
不是厂商认证语义；单位保持 `app_display_unit`，停止协议仍未确认。
