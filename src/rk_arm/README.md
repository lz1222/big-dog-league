# rk_arm：D1 基础驱动（安全默认关闭）

本包提供 D1 DDS 反馈订阅、原始状态和 ROS 服务接口。所有参数均为 **DEVELOPMENT DEFAULT / NOT HARDWARE VALIDATED**。

为避免 Unitree SDK 的 CycloneDDS 与 ROS Foxy/FastDDS 同进程混载，`d1_dds_driver_node` 是纯 Unitree DDS reader，使用本机 Unix datagram 将原始状态交给 `arm_manual_control_node`。后者在 `ROS_DOMAIN_ID=42` 发布 ROS 状态和服务；两个进程均没有 command writer。

## 当前安全状态

- `manual_motion_enabled: false`；驱动不会创建 `rt/arm_Command` writer。
- `sensor_msgs/JointState` 不发布：反馈单位仅为 `app_display_unit`，不能冒充弧度。
- `ArmRawState` 发布原始七通道、状态帧、反馈年龄与双源一致性。
- 关节与夹爪服务均经同一安全核心，默认返回 `MANUAL_MOTION_DISABLED`。
- 本地 `/tmp/rk_d1_arm_writer.lock` 的实现已就绪；它不能证明 DDS 网络不存在远程 writer。
- 运行 ROS CLI 时必须设置 `ROS_DOMAIN_ID=42`，例如 `export ROS_DOMAIN_ID=42`。

## 已知协议证据与边界

- `rt/arm_Feedback` 的 `funcode=1`：`angle0` 至 `angle6`；`funcode=3`：`enable_status`、`power_status`、`error_status`。来源：已提交的 `rk_arm_feedback_probe` 静态与现场采集。
- `current_servo_angle` 的 `PubServoInfo_`：`servo0_data_` 至 `servo6_data_`。来源：`third_party/unitree_d1_sdk/src/get_arm_joint_angle.cpp`。
- 单关节 JSON 外形 `funcode=1,data{id,angle,delay_ms}` 仅作为离线编码测试实现；来源：`third_party/unitree_d1_sdk/src/joint_angle_control.cpp`。
- SDK 中 `funcode=7` 被命名为 `arm_zero_control`，并无它是安全停止的证据。因此停止协议为 `STOP_SCHEMA_UNCONFIRMED`，服务绝不假成功。
- App 反馈映射由现场操作者确认：App 关节1～6对应 `angle0～5`/`servo_0～5`；夹爪对应第七路。状态：`USER-CONFIRMED APP-CONSISTENT MAPPING`，单位仍为 `APP DISPLAY UNIT`。

## 离线 App 命令模型

`d1_observed_command.hpp` 只实现 **OBSERVED FROM OFFICIAL APP TRAFFIC** 的
`funcode=2/address=1/mode=0` 完整七通道字符串解析、再编码和比较。它不是厂商
协议认证，不能推断 `address`、`mode` 或 `seq` 的正式语义。

影子生成器要求双源一致、未过期的七路反馈，复制所有当前 App 显示值后只修改
一个关节槽位（0～5）或夹爪槽位（6）。输出总带有
`DRY_RUN_ONLY / NOT SENT`，并拒绝非有限值、缺失/过期反馈、来源不一致、越界
索引和 `uint64_t` 序号溢出风险。它没有 DDS、停止、使能或失能 API；即使离线
JSON 可复现，`manual_motion_enabled` 也必须保持 `false`。

停止、保持、取消与失能候选的来源和边界见 [D1_STOP_PROTOCOL_AUDIT.md](D1_STOP_PROTOCOL_AUDIT.md)。

## 禁止范围

不包含视觉坐标、逆运动学、MoveIt、自动抓取、固定轨迹、`cmd_vel` 或 Action server。
