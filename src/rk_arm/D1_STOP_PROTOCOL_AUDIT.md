# D1 停止协议专项审计

状态：`STOP_SCHEMA_UNCONFIRMED`。本审计不产生、重放或发送 DDS 命令；所有结论都保持 fail-closed。

## 范围与来源

- 官方 App 被动流量：`artifacts/d1_command_protocol/`（Git 忽略）。已观测到 `funcode=2`；松开后未观察到独立 JSON。
- SDK 源码：`third_party/unitree_d1_sdk/src/`。
- 本机服务：`systemctl list-unit-files` 仅出现 `unitree-upgrade.service`，其启动 `/upgradePythonServer/server.py`；未找到 D1/arm command systemd 服务或其配置。
- 本机历史日志：本仓库只读 bridge 日志无停止或官方服务协议记录。
- 二进制字符串：历史 `get_arm_joint_angle` 可执行文件带有通用 DDS reader/writer 库符号，但未提供可归属的 `rt/arm_Command` 停止 JSON；链接符号不是协议行为证据。
- 官方公开产品页仅说明 D1 是 6 轴加夹爪、具备位置/速度/力控能力；没有公开 ArmString `funcode`、停止、保持或 TTL 格式。[D1-T 产品页](https://www.unitree.com/D1-T/)

## 候选方式

| 候选 | 原始来源 / JSON | 调用上下文 | App 使用 / 真机观察 | 停止后保持、失能或下落 | 结论 |
|---|---|---|---|---|---|
| 明确 Stop | 未找到 | 无 | 未观察 | 未知 | 不存在已确认 JSON。 |
| Hold / 保持位置 | `d1_joint5_control_diagnosis.cpp` 的 `hold_sec` 是诊断程序的本地观察等待，并非 ArmString 字段 | 诊断工具等待 | 未观察 | 未知 | 不是 hold 命令。 |
| 取消队列 | 未找到 JSON、服务或文档 | 无 | 未观察 | 未知 | 未确认。 |
| 速度归零 | App `funcode=2` 流无速度字段；产品页泛称支持速控 | 无 D1 ArmString 格式 | 未观察 | 未知 | 不可从缺失字段推断。 |
| `funcode=5` | `joint_enable_control.cpp:17`：`{"seq":4,"address":1,"funcode":5,"data":{"mode":0}}` | 示例文件名暗示关节使能 | 混合人工使能/夹爪窗口中见 `mode=0/1`，未隔离 | 未知，可能改变力矩状态 | 只记录候选使能相关；不得调用。 |
| `funcode=7` | `arm_zero_control.cpp:17`：`{"seq":4,"address":1,"funcode":7}`；抓取样例在轨迹前后以“零位”注释调用 | 零位/轨迹上下文 | App 未观察 | 未知 | 不是停止、取消或安全保持证据。 |
| App 松开 | 被动流量在人工操作后停止；无不同 `funcode` 或额外 JSON | App 摇杆操作 | 已观察到流停止，但标记存在人工偏差 | 未知 | `APP_RELEASE_STOPS_COMMAND_STREAM` 不成立。 |
| 退出页面 / App 断开 | 退出页面窗口为 0 command；正常断开未做 | 页面行为 | 退出已观察；断开未观察 | 未知 | 两者均不等于安全停止。 |

## 结论

1. 未找到明确 Stop、Hold、取消队列、速度归零、TTL 或 command stream 断开后设备行为的 D1 官方协议证据。
2. `funcode=5` 的名称和单个 `mode=0` SDK 样例不足以证明 enable/disable 的方向、后果或安全性；混合 App 记录也不能补足该语义。
3. `funcode=7` 的源码名称/注释是“zero”，不是安全停止认证；它可能产生运动，绝不复用为 Stop。
4. 因此 `StopArm` 必须继续返回 `STOP_SCHEMA_UNCONFIRMED`，而 `JogArmJoint` 与 `SetArmGripper` 继续返回 `MANUAL_MOTION_DISABLED`。
5. 任何未来验证只能先由官方 App 的明确停止/断开操作进行被动抓取；在获得可重复真机证据和独立安全评审前，不得创建 `rt/arm_Command` writer。
