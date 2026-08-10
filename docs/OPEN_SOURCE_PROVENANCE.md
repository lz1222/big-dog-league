# 开源来源与许可证记录

本文件是 `GO2_OPEN_SOURCE_MAZE_MIGRATION_V1` Phase A 的许可证门禁记录。审查日期为
2026-08-08；本阶段**没有复制、改编或链接任何下列仓库的源码**，因此 `target file` 均为
“无”。后续每次真正移植前必须补充精确的源文件、目标文件、方式、提交 SHA 与署名要求。

| 来源仓库 | 固定版本 | 根许可证 / 声明 | 本阶段用途 | 复制结论 |
|---|---|---|---|---|
| `jizhang-cmu/autonomy_stack_go2` | `foxy-humble` / `43d5f54b389b251713f0097893c30fa76c870d54` | 根目录无 `LICENSE`；`local_planner`、`terrain_analysis` 的 `package.xml` 写 BSD；`transform_sensors`、随附 `go2_sport_api`、`unitree_api` 写 `TODO: License declaration` | 架构、轨迹库和数据流参考 | **不得复制**。根仓库与关键子包的授权不完整；即使个别包声明 BSD，也须在复制前取得作者/维护者对该具体文件的可再授权确认。 |
| `unitreerobotics/unitree_ros2` | `master` / `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` | 根 `LICENSE`: BSD 3-Clause；`unitree_go`、`unitree_api` 亦声明 BSD 3-Clause | 官方 DDS、消息定义、Topic 与 API 事实来源 | 可以在后续按 BSD-3-Clause 保留版权、许可与免责声明后作小范围适配；本阶段未复制。 |
| `unitreerobotics/point_lio_unilidar` | `main` / `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` (`v2.0.2`) | 根 `LICENSE`: GPL-2.0；`package.xml` 的 BSD 字段与根许可证不一致，按更明确且更严格的根 GPL-2.0 处理 | 时间同步、去畸变、LIO 架构审查 | **不得复制/链接进当前闭源或许可证未知的工作区**。仅可独立重新实现不受表达保护的算法思想，并在实现记录中注明“inspired by”。 |

## 允许的后续记录格式

在 Phase B 以后，任何外部内容都必须先加入一行：

| Source repo | Source commit | Source file | Target file | copied / adapted / inspired | Attribution / license action |
|---|---|---|---|---|---|

当前记录：无待署名代码；无外部源码进入 `src/`；无新增第三方二进制或构建依赖。

