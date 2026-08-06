# D1 App 命令最小 fixture

这些 JSON 由 Git 忽略的 `artifacts/d1_command_protocol/` 被动官方 App 抓包最小化而来，
只保留 `seq`、`address`、`funcode`、`mode` 和七个 `angleN` 字段。它们不包含设备
序列号、网络地址、wall clock、操作员事件或整段日志。

| Fixture | 被动抓包来源 | 代表字段变化 |
|---|---|---|
| `joint1_positive.json` | `joint1_1`，seq 60406 | `angle0` 正向目标 |
| `joint1_negative.json` | `joint1_return_1`，seq 60430 | `angle0` 反向目标 |
| `joint2_positive.json` | `joint2_1`，seq 60461 | `angle1` 正向目标 |
| `gripper_open.json` | `gripper_open_1`，seq 60465 | `angle6` 打开目标 |
| `gripper_close.json` | `gripper_close_1`，seq 60477 | `angle6` 关闭目标 |

数值单位仍为 `app_display_unit`。这些 fixture 证明的是
**OBSERVED FROM OFFICIAL APP TRAFFIC** 的字符串外形，不是可发送或厂商认证协议。
