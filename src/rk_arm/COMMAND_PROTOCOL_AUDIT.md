# D1 ArmString 命令协议审计

状态：`manual_motion_enabled=false`、`COMMAND_ID_SEMANTICS_UNCONFIRMED`、`STOP_SCHEMA_UNCONFIRMED`、`value_unit=app_display_unit`。本报告只引用静态源码和被动 DDS 订阅；未创建、启动、测试或重放任何自研 `rt/arm_Command` writer。

**OBSERVED FROM OFFICIAL APP TRAFFIC，不是厂商正式协议认证。**

## 2026-08-07 官方 App 被动抓取

原始数据位于 Git 忽略的 `artifacts/d1_command_protocol/`。探针只创建 `ChannelSubscriber`，并同时保存 `rt/arm_Command`、`rt/arm_Feedback` 与 `current_servo_angle`。人工事件通过同机 Unix socket 写入；事件工具使用 `CLOCK_MONOTONIC`，probe 收到后立即记录自身单调时间并 flush。

### 时间同步校准

- `calibration_3`：`CALIBRATION_START→END` 的发送端间隔为 3.008 秒，probe 接收端间隔为 2.991 秒；两次本机 socket 接收延迟约为 19.9 ms 和 3.8 ms。此前约十秒的聊天事件偏差已消除。
- 事件仍表示人工标记时间，而非 App 内部动作时刻。因此，若标记和实际松开间隔明显，不能以该记录确认命令—反馈延迟或独立停止协议。

- `idle_1`：约 79 秒，App 空闲时 `rt/arm_Command` 为 0 帧；反馈维持约 18.95 Hz / 8.99 Hz。该观察窗口内没有空闲周期命令。
- `joint1_1`：App 关节1正向人工操作时收到 24 帧，`funcode=2`、`address=1`、`data.mode=0`。`seq=60406..60429` 连续递增；`angle0=1.4..24.4` 单调增加，其他目标字段近似不变。
- `joint1_return_1`：反向回位时收到 24 帧，字段外形相同，`seq=60430..60453` 紧接递增；`angle0=23.3..0.9` 下降。反馈持续，`exec_status=1`、`recv_status=1` 被观测到。
- `joint2_1` / `joint2_return_1`：App 关节2正、反向各收到两帧，均为 `funcode=2`、`address=1`、`mode=0`；`seq=60461..60464` 紧接此前会话。正向仅 `angle1` 目标由 `-88.3` 至 `-84.3`，反馈 `angle1/servo_1` 约由 `-90.3` 至 `-84.5`；反向目标至 `-87.5`，反馈至约 `-87.7`。人工摇杆无法精准回位，残差约 `+2.6 app_display_unit`，未执行重复组。
- `gripper_open_1`：12 帧 `funcode=2`、`address=1`、`mode=0`，`seq=60465..60476`；只有 `angle6` 由约 `-15.8` 递增至 `28.2`，反馈 `angle6/servo_6` 由 `-19.8` 到 `20.0`。
- `gripper_close_1`：22 帧同外形，`seq=60477..60498`；只有 `angle6` 由 `20` 递减至 `-30.8`，反馈 `angle6/servo_6` 由 `20.0` 到 `-19.8`。
- 在关节2及夹爪窗口的事件结束标记后均未再观察到命令帧，且无额外 JSON；但最后命令比人工标记早约 0.4–5 秒。因此，只能记录操作后的命令流为空，不能标记 `APP_RELEASE_STOPS_COMMAND_STREAM`，更不能确认停止协议。
- `app_page_exit_1`：退出控制页面后，观察窗口内没有 `rt/arm_Command`。重新进入页面后的记录混入人工使能切换及夹爪操作，出现 `funcode=5`（`mode=0`、`mode=1`）和 `funcode=2`；该混合窗口不能解释 `funcode=5`、页面进入行为或使能语义。App 断开未执行。

由此可更新的结论：

- `COMMAND_FUNCODE_2_OBSERVED_FOR_APP_JOG`：本机官方 App 对关节1、关节2和夹爪均实际使用全七通道目标 JSON，`mode=0`；其控制语义、完整安全边界仍未验证。
- `SEQ_CANDIDATE_MONOTONIC_COUNTER_WITHIN_OBSERVED_SESSION`：`seq=60406..60498` 跨关节1、关节2和夹爪严格连续，无跳号；退出并重新进入 App 后、混入人工使能/夹爪操作的记录继续为 `60499..60503`。App 正常断开与重新连接均未执行，不能宣称全局递增、重连不重置或自研程序必须生成该序号，故仍是 `COMMAND_ID_SEMANTICS_UNCONFIRMED`。
- `address` 在关节1、关节2和夹爪记录中均为 1；本轮不支持 `COMMAND_ADDRESS_CANDIDATE_ACTUATOR_INDEX`。
- 此处无 `id` 字段，不能用 App 观测确认 SDK `funcode=1.data.id` 的语义；`funcode=1` 在本批 App 数据中未出现。
- 命令 `angle0`、`angle1` 和 `angle6` 分别与相应反馈通道的变化方向和值域相符，仅可写为 `COMMAND_VALUE_MATCHES_APP_DISPLAY_UNIT`，不能转换为 degree 或 radian。
- 打开、关闭夹爪的可重复命令外形已观察到，可记录 `GRIPPER_COMMAND_SCHEMA_OBSERVED`；这不授权自研夹爪控制。
- 未观测 `delay_ms`、速度字段、明确 App 停止命令或 App 断开行为；`STOP_SCHEMA_UNCONFIRMED` 不变。

离线命令模型、最小化 fixture 和影子生成器见 `d1_observed_command.hpp`。它只接受完整七通道的 App 已观测外形，所有结果均为 `DRY_RUN_ONLY / NOT SENT`；它不包含 DDS writer、停止、使能或失能生成入口。专项停止审计见 [D1_STOP_PROTOCOL_AUDIT.md](D1_STOP_PROTOCOL_AUDIT.md)。

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
