# D1 ArmString 命令协议审计

状态：`COMMAND_ID_SEMANTICS_UNCONFIRMED`、`STOP_SCHEMA_UNCONFIRMED`。本报告只引用静态源码和已完成的只读反馈验收；未抓取、重放或发送 App command。

## 2026-08-07 官方 App 被动抓取

原始数据位于 Git 忽略的 `artifacts/d1_command_protocol/`。探针只创建 `ChannelSubscriber`，并同时保存 `rt/arm_Command`、`rt/arm_Feedback` 与 `current_servo_angle`。

- `idle_1`：约 79 秒，App 空闲时 `rt/arm_Command` 为 0 帧；反馈维持约 18.95 Hz / 8.99 Hz。该观察窗口内没有空闲周期命令。
- `joint1_1`：App 关节1正向人工操作时收到 24 帧，`funcode=2`、`address=1`、`data.mode=0`。`seq=60406..60429` 连续递增；`angle0=1.4..24.4` 单调增加，其他目标字段近似不变。
- `joint1_return_1`：反向回位时收到 24 帧，字段外形相同，`seq=60430..60453` 紧接递增；`angle0=23.3..0.9` 下降。反馈持续，`exec_status=1`、`recv_status=1` 被观测到。
- 操作事件标记与真实 App command 开始相差约十秒，故不能为本批数据声明精确 command-to-feedback 延迟或命令停止后的稳定时间。
- 因人工最小脉冲实际形成约 24 帧命令、目标变化约 23 个 App 显示单位，本轮停止后续物理操作；未采集关节2、夹爪、App 明确停止、退出页面或断开。

由此可更新的结论：

- `funcode=2` 为本机官方 App 关节1在观测会话中实际使用的全七通道目标 JSON，`mode=0`；其控制语义、完整安全边界仍未验证。
- `seq` 在同一 App 会话、同一关节的连续帧中递增；尚未比较其他关节、夹爪或重连，故仍是 `COMMAND_ID_SEMANTICS_UNCONFIRMED`。
- 此处无 `id` 字段，不能用 App 观测确认 SDK `funcode=1.data.id` 的语义；`funcode=1` 在本批 App 数据中未出现。
- 命令 `angle0` 与第 0 路反馈的变化方向和值域相符，仅可写为 `COMMAND VALUE MATCHES APP DISPLAY UNIT`，不能转换为 degree 或 radian。
- 未观测 `delay_ms`、速度字段、独立夹爪命令或明确 App 停止命令；`STOP_SCHEMA_UNCONFIRMED` 不变。

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
