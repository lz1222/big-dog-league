# 迷宫测试证据索引

任何没有对应 bag、日志、视频或人工记录的项目均保持 `UNVERIFIED`。

| 测试编号 | rosbag | 视频 | 日志/预测 | 参数文件 | Git SHA | 起点/终点照片 | 实际最小间隙 | 接触 | 结果 |
|---|---|---|---|---|---|---|---|---|---|
| ROUND15_PHYSICAL | `/home/unitree/maze_bags/b2_round15_20260802_144706` | 未登记 | `docs/B2_MAZE_NAVIGATION_POLICY_DRY_RUN.md` | `config/maze_navigation_dry_run.yaml` | `f69616e`附近 | 未登记 | 转后左侧约9cm，过程中轻触 | 左侧机身/内侧板 | FAIL |
| ROUND15_LEGACY_REPLAY | 同上 | 不适用 | 旧B2约4.52deg、left约0.362m触发 `turn_sweep_unsafe` | `config/maze_navigation_dry_run.yaml` | `f69616e`附近 | 不适用 | 固定0.413m粗门限 | 不执行 | DRY_RUN_PASS |
| B2_1_A_ROUND15_REPLAY_V2 | `/home/unitree/maze_bags/b2_round15_20260802_144706` | 不适用 | 已生成 `b2_1_a_round15_geometry_summary_v2.json`；左侧匹配 round15 但 gate FAIL | `config/maze_round15_replay.yaml`、`config/maze_first_turn_dry_run.yaml` | `cb7aa7a` | 不适用 | 左侧 0.049m @23deg，但右侧墙端假阳性 | 不执行 | FAIL（gate: 左侧墙段null + 右侧假阳性） |
| B2_1_A_SYNTHETIC_REPLAY_UNIT | 无，纯逻辑合成数据 | 不适用 | `tests/test_maze_round15_replay_core.py` | 测试内固定几何 | 未提交 | 不适用 | 合成旧左弧被提前拒绝 | 不执行 | UNIT_TEST_PASS |
| B2_1_A_STATIC_EMPTY | `/home/unitree/maze_bags/b2_1_a_static_empty_20260803_004235` | 不适用 | 182s、2679点云帧、365 dry_run_status。14.72Hz构图、motion_output全False、后向不足、空场景无碰撞 | `config/maze_first_turn_dry_run.yaml` | `cb7aa7a` | 不适用 | 空场景，无碰撞 | 不执行 | HARDWARE_SINGLE_PASS |
| B2_1_A_STATIC_MAZE_ENTRY | `/home/unitree/maze_bags/b2_1_a_static_maze_entry_20260803_011605` | 不适用 | 137s、2020点云帧。**无dry_run_status输出**（节点未运行） | `config/maze_first_turn_dry_run.yaml` | `cb7aa7a` | 不适用 | 仅原始传感器数据 | 不执行 | UNVERIFIED |
| B2_1_A_STATIC_MAZE_ENTRY_RETRY | `/home/unitree/maze_bags/b2_1_a_static_maze_entry_retry_20260803_170707` | 不适用 | 58s、863点云帧、117 dry_run_status。FAULT_STOP（右侧wall_endpoint碰撞）、2墙段、motion_output全False、后向全不足 | `config/maze_first_turn_dry_run.yaml` | `cb7aa7a` | 不适用 | 右侧wall_endpoint静态碰撞(0.25,-0.24) | 不执行 | HARDWARE_SINGLE_PASS |

## Round15 已知物理事实

- 实际接触约发生在左转 `44deg`。
- 接触附近：`front=0.487m`、`left_front=0.647m`、`left=0.393m`、
  `right=0.263m`。
- 转前人工间隙：左 `15/17cm`、右 `24/22cm`。
- 转后人工间隙：左 `9/9cm`、右 `27/27cm`。
- 接触部位：左侧机身；挡板：转弯内侧板；程度：轻触。

这些值只作为回放验收对照，禁止硬编码进碰撞算法或单元测试答案。
