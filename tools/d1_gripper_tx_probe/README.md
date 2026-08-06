# d1_gripper_tx_probe — EXPERIMENTAL HARDWARE WRITER

此目录不属于 `rk_arm` 正式驱动、比赛 launch 或 ROS 图。它是为未来单次夹爪
验证准备的隔离实验程序；默认 CMake 配置会失败，必须明确开启：

```bash
cmake -S tools/d1_gripper_tx_probe -B /tmp/d1_gripper_tx_probe_build \
  -DBUILD_D1_EXPERIMENTAL_WRITER=ON
cmake --build /tmp/d1_gripper_tx_probe_build -j2
ctest --test-dir /tmp/d1_gripper_tx_probe_build --output-on-failure
```

它只链接 Unitree SDK2/CycloneDDS，不链接 ROS、rclcpp 或 FastDDS。运行时默认
只建立 `rt/arm_Feedback` 和 `current_servo_angle` 的 reader。dry run 必须显式
给出序号，且不会创建 DDS writer 或本地 writer lock：

```bash
/tmp/d1_gripper_tx_probe_build/d1_gripper_tx_probe --interface eth0 --seq 60504
```

输出固定为 `DRY_RUN_ONLY / NOT SENT`，显示完整七路反馈、反馈年龄、双源状态、
状态帧、锁状态、delta、目标和完整 JSON。只允许 `target_angle6 = current_angle6 + delta`，
其中 `delta` 在代码中硬限制为 `[-1.0, 1.0] app_display_unit`；前六路由当前完整
反馈逐位复制，命令行没有关节索引、前六路目标、轨迹、循环、停止或使能选项。

未来硬件路径要求同时出现 `--hardware-send` 与
`--confirm SEND_ONE_GRIPPER_TARGET`，并在本地锁可用时才创建 writer。该路径最多
调用一次 `Write`，随后立即关闭 writer；没有重试、保持、回位或第二帧。**本阶段
不得执行该参数组合。** 本地锁只能防止本机进程冲突，不能证明网络没有远程 writer。

`address=1`、`mode=0` 与 `funcode=2` 都是 **OBSERVED FROM OFFICIAL APP TRAFFIC**，
不是厂商认证语义；单位保持 `app_display_unit`，`STOP_SCHEMA_UNCONFIRMED` 不变。
