# rk_arm：D1 基础驱动（安全默认关闭）

本包提供 D1 DDS 反馈订阅、原始状态和 ROS 服务接口。所有参数均为 **DEVELOPMENT DEFAULT / NOT HARDWARE VALIDATED**。

## 当前安全状态

- `manual_motion_enabled: false`；驱动不会创建 `rt/arm_Command` writer。
- `sensor_msgs/JointState` 不发布：反馈单位仅为 `app_display_unit`，不能冒充弧度。
- `ArmRawState` 发布原始七通道、状态帧、反馈年龄与双源一致性。
- 关节与夹爪服务均经同一安全核心，默认返回 `MANUAL_MOTION_DISABLED`。
- 本地 `/tmp/rk_d1_arm_writer.lock` 的实现已就绪；它不能证明 DDS 网络不存在远程 writer。

## 已知协议证据与边界

- `rt/arm_Feedback` 的 `funcode=1`：`angle0` 至 `angle6`；`funcode=3`：`enable_status`、`power_status`、`error_status`。来源：已提交的 `rk_arm_feedback_probe` 静态与现场采集。
- `current_servo_angle` 的 `PubServoInfo_`：`servo0_data_` 至 `servo6_data_`。来源：`third_party/unitree_d1_sdk/src/get_arm_joint_angle.cpp`。
- 单关节 JSON 外形 `funcode=1,data{id,angle,delay_ms}` 仅作为离线编码测试实现；来源：`third_party/unitree_d1_sdk/src/joint_angle_control.cpp`。
- SDK 中 `funcode=7` 被命名为 `arm_zero_control`，并无它是安全停止的证据。因此停止协议为 `STOP_SCHEMA_UNCONFIRMED`，服务绝不假成功。
- App 反馈映射由现场操作者确认：App 关节1～6对应 `angle0～5`/`servo_0～5`；夹爪对应第七路。状态：`USER-CONFIRMED APP-CONSISTENT MAPPING`，单位仍为 `APP DISPLAY UNIT`。

## 禁止范围

不包含视觉坐标、逆运动学、MoveIt、自动抓取、固定轨迹、`cmd_vel` 或 Action server。
