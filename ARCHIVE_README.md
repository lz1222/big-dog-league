# 2026-08-04 本地工作空间快照

本分支保存 `/home/unitree/rk_inspection_ws` 中 329 项本地文件和符号链接的
原始快照。快照来源分支为 `fix/sdk-server-dds-runtime-v1`，来源提交为
`d13a5d0ea8ab8edb52a23e1ea11d46052b690888`，采集日期为 2026-08-04。
原始载荷共 61,049,776 字节；原工作空间仍保留全部文件及原路径。

## 内容分类

- `Testing/`：CTest 临时状态和测试日志。
- `artifacts/`：现场验证图片、JSON 报告和运行日志。
- `inspection_action_executor.yaml`：未被当前启动链引用的遗留配置快照；正式
  配置已位于 `src/rk_bringup/config/non_arm_competition_params.yaml`。
- `maze_handoff_20260804/`：历史交接文档、离线回放结果和 bag metadata。
- `tools/`：legacy stair CLI 的 CMake/colcon 构建、安装、日志、二进制及
  符号链接快照；目录中没有可复用源码。

其中部分交接资料、CMake cache 和链接命令包含采集机器的绝对路径，只用于
历史审计和恢复，不应被当作可移植运行配置。该分支不建议合并到 `master`，
也不应用其中的二进制直接连接机器人。

## 清单与校验

- `archive_inventory.tsv`：329 项载荷的类型、大小、分类和处置依据。
- `archive_sha256sums.txt`：普通文件按文件内容计算 SHA256；符号链接按链接
  目标文本计算 SHA256。
- `archive_symlinks.tsv`：3 个符号链接的目标、权限和时间信息。

恢复时应在独立目录检出本分支，再使用保留权限、时间戳和符号链接的复制：

```bash
rsync -aH --ignore-existing <archive-worktree>/ <target-workspace>/
```

恢复前必须确认目标路径不存在同名冲突，且只复制上述五类载荷；不得使用
`--delete` 或 `--remove-source-files`。恢复后按清单逐项计算 SHA256，并核对
符号链接目标和执行位。
