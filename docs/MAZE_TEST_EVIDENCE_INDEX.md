# 迷宫测试证据索引

任何没有对应 bag、日志、视频或人工记录的项目均保持 `UNVERIFIED`。

| 测试编号 | rosbag | 视频 | 日志/预测 | 参数文件 | Git SHA | 起点/终点照片 | 实际最小间隙 | 接触 | 结果 |
|---|---|---|---|---|---|---|---|---|---|
| ROUND15_PHYSICAL | `/home/unitree/maze_bags/b2_round15_20260802_144706` | 未登记 | `docs/B2_MAZE_NAVIGATION_POLICY_DRY_RUN.md` | `config/maze_navigation_dry_run.yaml` | `f69616e`附近 | 未登记 | 转后左侧约9cm，过程中轻触 | 左侧机身/内侧板 | FAIL |
| ROUND15_LEGACY_REPLAY | 同上 | 不适用 | 旧B2约4.52deg、left约0.362m触发 `turn_sweep_unsafe` | `config/maze_navigation_dry_run.yaml` | `f69616e`附近 | 不适用 | 固定0.413m粗门限 | 不执行 | DRY_RUN_PASS |
| B2_1_A_ROUND15_REPLAY | `evidence/round15/b2_round15_20260802_144706/b2_round15_20260802_144706_0.db3`，SHA256 `6788e868...a7b3b19` | 不适用 | `/tmp/b2_1_a_round15_replay_summary.json`，SHA256 `a2399d...535162`；23.980deg 左侧风险提前 7.278s，有限短片段关联 | `config/maze_round15_replay.yaml`、`config/maze_first_turn_dry_run.yaml` | 工作区未提交 | 不适用 | 预测最小间隙 0；匹配左侧点 0.049m | 不执行 | DRY_RUN_PASS |
| B2_1_A_SYNTHETIC_REPLAY_UNIT | 无，纯逻辑合成数据 | 不适用 | `tests/test_maze_round15_replay_core.py` | 测试内固定几何 | `16251ae` | 不适用 | 合成旧左弧被提前拒绝 | 不执行 | UNIT_TEST_PASS |

## Round15 已知物理事实

- 实际接触约发生在左转 `44deg`。
- 接触附近：`front=0.487m`、`left_front=0.647m`、`left=0.393m`、
  `right=0.263m`。
- 转前人工间隙：左 `15/17cm`、右 `24/22cm`。
- 转后人工间隙：左 `9/9cm`、右 `27/27cm`。
- 接触部位：左侧机身；挡板：转弯内侧板；程度：轻触。

这些值只作为回放验收对照，禁止硬编码进碰撞算法或单元测试答案。
