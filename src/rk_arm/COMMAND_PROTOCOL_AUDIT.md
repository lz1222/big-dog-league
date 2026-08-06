# D1 ArmString 命令协议审计

状态：`COMMAND_ID_SEMANTICS_UNCONFIRMED`、`STOP_SCHEMA_UNCONFIRMED`。本报告只引用静态源码和已完成的只读反馈验收；未抓取、重放或发送 App command。

| 项目 | 证据 | 原始 JSON | 强度 / 结论 |
|---|---|---|---|
| ArmString 外层 | `third_party/unitree_d1_sdk/src/joint_angle_control.cpp:13-18` | `{"seq":4,"address":1,"funcode":1,"data":...}` | 中：SDK 示例实际将该字符串写至 `rt/arm_Command`。`seq`、`address` 的语义未证实。 |
| 单关节 JSON 外形 | `joint_angle_control.cpp:17`；`d1_joint5_control_diagnosis.cpp:214-239` | `{"seq":4,"address":1,"funcode":1,"data":{"id":5,"angle":60,"delay_ms":0}}` | 中：字段及类型已证实；`id` 是执行器编号、序号或其他语义未证实；`angle` 单位、绝对/相对语义和 `delay_ms` 行为未真机验证。 |
| 七通道目标 JSON 外形 | `multiple_joint_angle_control.cpp:17`；`get_arm_joint_angle.cpp:23-30` | `{"seq":4,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,...,"angle6":0}}` | 中：七个字段存在；多关节固定轨迹示例不得用于本驱动。`mode=1` 语义未证实。 |
| `funcode=5` | `joint_enable_control.cpp:17` | `{"seq":4,"address":1,"funcode":5,"data":{"mode":0}}` | 弱：文件名暗示 enable，但没有文档、反馈或真机观察证明 mode 值及使能/失能语义。不可调用。 |
| `funcode=7` | `arm_zero_control.cpp:17` 及抓取示例开头 | `{"seq":4,"address":1,"funcode":7}` | 弱：文件名仅表明 zero control；没有安全停止、取消队列、失能或保持当前位置的证据。不是停止命令。 |
| 夹爪命令 | `grasp_*.cpp` 中仅有 `funcode=2` 的 `angle6` 轨迹值 | 例如 `{"mode":1,...,"angle6":45.400002}` | 弱：与已确认的第七反馈通道相容，但不证明角度单位、目标语义或安全的独立夹爪格式。不可调用。 |
| 速度 / 周期发送 / command TTL | 审计的 JSON 示例均无速度字段，SDK 示例一次 `Write` | 无 | 未确认。不得将缺失字段解释为无限期保持或自动超时。 |
| 安全停止 | 没有官方文档、App command 抓包或低风险停止观察 | 无 | `STOP_SCHEMA_UNCONFIRMED`。 |

## 已完成的反馈证据

- 2026-08-07 只读 probe：`rt/arm_Feedback` 152 帧/8 秒（19.019 Hz），`current_servo_angle` 72 帧/8 秒（8.978 Hz），坏 JSON 为 0。
- 2026-08-07 分离后的 ROS 原始状态：161 帧/8 秒（20.000 Hz），`feedback_valid=true`、`sources_consistent=true`、`value_unit=app_display_unit`，状态组合为 `(enable_status,power_status,error_status)=(1,0,0)`。
- 现场操作者确认的反馈映射保留为 `USER-CONFIRMED APP-CONSISTENT MAPPING`；这不是命令协议或单位证据。

## 可解除未确认状态所需证据

仅可在操作者通过官方 App 单独执行明确动作时，被动监听 `rt/arm_Command` 并保存原始 ArmString 后补充。必须分别记录 App 连接、单关节最小动作、夹爪开/关、松开摇杆、明确停止和正常断开；严禁重放、修改或发送捕获结果。
