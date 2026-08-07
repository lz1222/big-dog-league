# 2026-08-04 本地工作空间归档索引

原工作空间中的 329 项本地文件和符号链接没有被删除、移动、重命名或修改，
仍保留在原始路径。GitHub 完整快照位于分支
`archive/local-workspace-snapshot-20260804`，最终快照提交为
`9f4b54ff213cd6ca0096433daa43e500f5c2fb39`。

## 分类与处置

本地引用、launch、启动脚本、安装规则和默认路径检查没有发现 A/B/C 类的
正式运行依赖、测试直接依赖或可复用源码。因此 fix 分支没有直接跟踪这 329
项中的任何文件：

- `Testing/`：CTest 临时状态和日志，仅在 archive 分支跟踪。
- `artifacts/`：现场验证图片、JSON 结果和日志，仅在 archive 分支跟踪。
- `inspection_action_executor.yaml`：未被启动链引用的遗留配置副本；当前正式
  配置位于 `src/rk_bringup/config/non_arm_competition_params.yaml`，关键路径
  和 smoke 参数由 launch 注入。
- `maze_handoff_20260804/`：历史交接文档、回放结果和 bag metadata，仅在
  archive 分支跟踪。
- `tools/`：legacy stair CLI 的 CMake/colcon 构建、安装、日志、二进制和
  符号链接快照；其中没有源码，仅在 archive 分支跟踪。

这些路径在当前 fix 分支由根目录 `.gitignore` 精准隐藏。这样不会把生成物、
本机绝对路径或历史二进制混入功能提交，同时本地运行和历史调试仍可访问原路径。

## 查看与恢复

查看远端快照：

```bash
git ls-tree -r --name-only \
  origin/archive/local-workspace-snapshot-20260804 -- \
  Testing artifacts inspection_action_executor.yaml maze_handoff_20260804 tools
```

恢复时应把 archive 分支检出到独立 worktree，先确认目标不存在同名冲突，再按
顶层目录使用 `rsync -aH --ignore-existing` 复制。禁止使用 `--delete` 或
`--remove-source-files`。archive 分支中的 `archive_inventory.tsv`、
`archive_sha256sums.txt` 和 `archive_symlinks.tsv` 用于核对 329 项内容、执行位
和符号链接目标。

归档分支用于保存本地工作空间快照，不建议合并到 `master`，其中的遗留二进制
也不应直接用于机器人控制。
