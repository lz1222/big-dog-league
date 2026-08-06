# D1 静止反馈协议审计报告

采集时间：2026-08-06 16:57:54Z 至 16:58:24Z（主机 wall time）
采集方式：`d1_feedback_probe --interface eth0 --duration-sec 30`
范围：仅 DDS reader；本次未创建控制通道、未操作机械臂或夹爪。

## 运行环境

- 分支：`maze-fusion-t0-t3`
- 基线：`24767fa900705878c3e3fa2b6b1b750fdc33dbcb`
- 网卡：`eth0`，采集前状态为 UP，IPv4 `192.168.123.18/24`。
- 工具为独立 CMake C++ 程序，未 source ROS 或工作区 install，也未启动 ROS daemon。
- 实际动态链接：`libddsc.so.0` 与 `libddscxx.so.0` 均来自 `third_party/unitree_sdk2/thirdparty/lib/aarch64/`；可执行文件具有同目录 DT_RPATH，未混用 Foxy 或系统 DDS 库。

## Topic 观测

| Topic | 收到 | 帧数 | 首帧等待 | 平均频率 | 坏帧 | 结束前 stale |
|---|---:|---:|---:|---:|---:|---:|
| `rt/arm_Feedback` | 是 | 569 | 75.69 ms | 18.97 Hz | 0 | 否（最后帧距结束 35.18 ms） |
| `current_servo_angle` | 是 | 270 | 81.18 ms | 8.99 Hz | 0 | 否（最后帧距结束 50.66 ms） |
| `rt/current_servo_angle` | 否 | 0 | 无 | 0 Hz | 0 | 是 |

stale 阈值为开发默认值 1000 ms，未经过硬件验证。第三个 topic 在整个窗口无帧，记录为 stale；前两个 topic 在 reader 正常关闭前持续到帧，因此本轮没有观察到其发布端消失后的 stale 转换。

## `rt/arm_Feedback` 原始 JSON 观测

每个 payload 顶层都是 JSON object，所有 569 帧都有 `seq`、`address`、`funcode`、`data`。未观察到 `timestamp`、`stamp`、`time_ns` 或 `time_ms` 类设备字段；`seq` 在本轮恒为 10，不能作为递增序列的证据。

本轮有两种 `funcode` 与 `data` 形状：

| `funcode` | 帧数 | `data` 字段 | 观测范围 |
|---:|---:|---|---|
| 1 | 270 | `angle0` 至 `angle6` | 见下表 |
| 3 | 299 | `enable_status`、`power_status`、`error_status` | 依次恒为 1、0、0 |

原始 payload 例子：

```json
{"seq":10,"address":2,"funcode":3,"data":{"enable_status":1,"power_status":0,"error_status":0}}
```

```json
{"seq":10,"address":2,"funcode":1,"data":{"angle0":0.5,"angle1":-90.30000305175781,"angle2":90.30000305175781,"angle3":-1.0,"angle4":3.299999952316284,"angle5":0.10000000149011612,"angle6":-19.799999237060547}}
```

payload 原始长度范围为 95 至 240 字节，均被完整保存在 JSONL。容错 JSON 解析坏帧数为 0。

## 七路原始 float 静止值

`current_servo_angle` 的 270 帧包含 7 个 float，数值范围如下。字段名只表示消息槽位，未赋予关节、夹爪或单位语义。

| 槽位 | 首帧 | 最小 | 最大 | 范围 |
|---|---:|---:|---:|---:|
| `servo_0` | 0.500000 | 0.400000 | 0.500000 | 0.100000 |
| `servo_1` | -90.300003 | -90.300003 | -90.199997 | 0.100006 |
| `servo_2` | 90.300003 | 90.300003 | 90.400002 | 0.099998 |
| `servo_3` | -1.000000 | -1.100000 | -0.900000 | 0.200000 |
| `servo_4` | 3.300000 | 3.200000 | 3.300000 | 0.100000 |
| `servo_5` | 0.100000 | 0.000000 | 0.200000 | 0.200000 |
| `servo_6` | -19.799999 | -19.799999 | -19.799999 | 0.000000 |

`funcode=1` 的 `angle0` 至 `angle6` 范围与这些槽位逐项一致。本轮仅证明两个反馈载荷具有相同的静止数值；不能确认角度单位、正负方向、零点、物理关节编号或第七路的物理含义。

## 证据、验证与边界

- 原始证据：`artifacts/d1_feedback_probe/arm_feedback_raw.jsonl`、`servo_angle_raw.csv`、`protocol_summary.json`。根 `.gitignore` 排除了 `artifacts/`，不会自动进入 Git 状态。
- 离线测试通过：正常/空 JSON、非法 JSON、类型与数组长度变化、超长 payload、非有限数字文本、连续坏帧、七路浮点落盘、stale 计时和文件关闭。
- 静态扫描未发现禁止命令 topic、发送 API 或 DDS 控制 writer 标识；通信源码只引用三个 reader 的订阅和关闭 API。
- 本轮没有速度、温度、设备时间戳或明确故障字段的实测证据。`*_status` 仅按原始字段名记录，未推断其协议语义。

本阶段到此停止。下一轮若获明确确认，应仅设计一次极小、可中止的动作实验，以建立槽位与实际运动的映射；在此之前不实现或运行任何机械臂控制。
