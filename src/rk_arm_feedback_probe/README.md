# D1 纯反馈协议探针

本目录是独立 CMake C++ 工具，不是 ROS 2 包。选择独立进程的原因是：当前机器同时具有 Foxy Fast DDS、Foxy CycloneDDS、系统 CycloneDDS 和 Unitree SDK 携带的 CycloneDDS；本工具不需要 ROS 图，也不应启动 ROS daemon。它只使用 Unitree SDK 的接收通道，并将实际 SDK 自带 DDS 目录设为 RPATH。

工具同时尝试订阅下列 topic，但不会把任一 topic 的存在视为前提：

- `rt/arm_Feedback`：完整保存字符串到 JSONL，并对有效 JSON 生成无语义 schema 摘要。
- `current_servo_angle`：保存 7 个原始 float 到 CSV。
- `rt/current_servo_angle`：保存 7 个原始 float 到 CSV。

程序不会推断角度单位、7 路排列、最后一路物理含义，或 JSON 字段的控制含义。输出中的路径、类型、数值范围都只是观测结果。

## 构建与静止采集

在不 source ROS 或工作区 install 的独立终端运行。先确认机械臂静止、周围无人，并确认没有其他测试工具运行：

```bash
cd /home/unitree/rk_inspection_ws
cmake -S src/rk_arm_feedback_probe -B /tmp/d1_feedback_probe_build
cmake --build /tmp/d1_feedback_probe_build -j2
ctest --test-dir /tmp/d1_feedback_probe_build --output-on-failure
env -u LD_LIBRARY_PATH /tmp/d1_feedback_probe_build/d1_feedback_probe \
  --interface eth0 --duration-sec 30
```

`--interface` 应替换为实际连接 D1 DDS 的网卡；空值使用 SDK 默认接口。程序只在 30 秒静止采集结束或收到 Ctrl-C 后关闭 reader 并刷新本地文件。

采集输出位于 `artifacts/d1_feedback_probe/`，其中原始数据被该目录的 `.gitignore` 排除。它包含：

- `arm_feedback_raw.jsonl`：接收时间、topic、完整原始字符串和字节长度。
- `servo_angle_raw.csv`：接收时间、topic 与 7 路原始 float。
- `protocol_summary.json`：每个 topic 的到帧/变化/坏帧/stale 统计及 JSON 字段摘要。

运行前后都应记录实际链接库，若发现不是 SDK aarch64 目录的 `libddsc.so` / `libddscxx.so`，停止采集并处理 DDS 环境隔离问题：

```bash
ldd /tmp/d1_feedback_probe_build/d1_feedback_probe | grep -E 'ddsc|dds|unitree|rcl|rmw'
```

本阶段结束条件是静止反馈采集和协议报告，不进行任何动作采集或控制开发。
