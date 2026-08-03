# 迷宫项目验收矩阵

状态只使用：`NOT_STARTED`、`IMPLEMENTED`、`UNIT_TEST_PASS`、
`DRY_RUN_PASS`、`HARDWARE_SINGLE_PASS`、`HARDWARE_REPEAT_PASS`、
`FULL_CHAIN_PASS`、`FAIL`、`BLOCKED`、`UNVERIFIED`。

| 阶段 | 检查项 | 要求 | 实际结果 | 状态 | 证据 | 日期 | Git SHA |
|---|---|---|---|---|---|---|---|
| B0 | PointCloud2 | `/utlidar/cloud_base`, base_link, 持续可用 | 历史真机约14.7Hz | HARDWARE_SINGLE_PASS | `docs/B0_MAZE_SENSOR_CALIBRATION.md` | 2026-07 | 历史提交 |
| B0 | Odom | `odom -> base_link`，持续可用 | 历史真机约149Hz | HARDWARE_SINGLE_PASS | `docs/B0_MAZE_SENSOR_CALIBRATION.md` | 2026-07 | 历史提交 |
| B1 | 五扇区 | 首弯静态距离不退化 | 已完成基础标定 | HARDWARE_SINGLE_PASS | `docs/B1_MAZE_PERCEPTION_DRY_RUN.md` | 2026-07 | 历史提交 |
| B1.5 | Move/StopMove | 稳定迈步和停车 | `0.25m/s` 10/10，约20cm，无尾移 | HARDWARE_REPEAT_PASS | `docs/B1_5_GO2_DETERMINISTIC_UDP_MOTION_BRIDGE.md` | 2026-07 | 历史提交 |
| B1.5 | watchdog/急停 | 故障可停车且不重启 | 用户报告通过 | HARDWARE_SINGLE_PASS | 对话记录，待补外部文件 | 2026-07 | 历史提交 |
| B2 | Round15物理安全 | 第一弯零接触 | 左侧机身轻触内侧板 | FAIL | `/home/unitree/maze_bags/b2_round15_20260802_144706` | 2026-08-02 | `f69616e`附近 |
| B2 | 0.413m粗保护 | 旧轨迹接触前停止 | 旧B2重放约4.52deg触发 | DRY_RUN_PASS | `docs/B2_MAZE_NAVIGATION_POLICY_DRY_RUN.md` | 2026-08-02 | `f69616e`附近 |
| B2.1-A | 局部占据图 | 每帧地图、过滤、约10Hz | 合成场景构图约30ms；真机频率未测 | UNIT_TEST_PASS | `tests/test_maze_first_turn_core.py` | 2026-08-02 | `16251ae` |
| B2.1-A | 后向三扇区 | 距离/点数/年龄/覆盖 | 代码已实现，真机覆盖未测 | UNIT_TEST_PASS | `tests/test_maze_first_turn_core.py` | 2026-08-02 | `16251ae` |
| B2.1-A | 后退失效保护 | 后方不足明确禁用 | 纯逻辑测试通过 | UNIT_TEST_PASS | `tests/test_maze_first_turn_core.py` | 2026-08-02 | `16251ae` |
| B2.1-A | 墙线/墙端 | 有限段、端点、残差、置信度、原始点 | 完整墙段与只读短片段分层；短片段不能授权转弯 | UNIT_TEST_PASS | `tests/test_maze_first_turn_core.py` | 2026-08-02 | 工作区未提交 |
| B2.1-A | 动态矩形连续扫掠 | 全轨迹、尾程、不确定性、危险部位 | 解析墙段、墙端和原始点测试通过 | UNIT_TEST_PASS | `tests/test_maze_first_turn_core.py` | 2026-08-02 | `16251ae` |
| B2.1-A | 六级分级/前5排名 | 安全优先，不以速度抵消风险 | 六候选及排序纯逻辑测试通过 | UNIT_TEST_PASS | `tests/test_maze_first_turn_core.py` | 2026-08-02 | `16251ae` |
| B2.1-A | Round15回放工具 | 历史地图+后续实际Odom连续检查 | 合成旧弧与短片段证据关联测试通过 | UNIT_TEST_PASS | `tests/test_maze_round15_replay_core.py` | 2026-08-02 | 工作区未提交 |
| B2.1-A | Round15新几何重放 | 44deg接触前输出完整几何FAIL及对应墙段 | `left_side` 预测约23.980deg、领先7.278s；关联有限短片段，不能授权轨迹 | DRY_RUN_PASS | `/tmp/b2_1_a_round15_replay_summary.json` | 2026-08-02 | 工作区未提交 |
| B2.1-A | 无运动输出 | 无速度或Unitree发布器 | 静态检查仅String发布器 | UNIT_TEST_PASS | `scripts/maze_first_turn_dry_run.py` | 2026-08-02 | `16251ae` |
| B2.1-B | 步态冻结 | 模式和状态验证、运动模型VERIFIED | 未开始 | NOT_STARTED |  |  |  |
| B2.1-C | 自动短段 | 直行/后退人工逐次5/5 | 未开始 | NOT_STARTED |  |  |  |
| B2.1-D | 第一弯单次 | 30项全部满足 | 未开始 | NOT_STARTED |  |  |  |
| B2.1-E | 第一弯连续 | 相同配置连续3次 | 未开始 | NOT_STARTED |  |  |  |
| B2.2 | 右弯及五弯 | 各自连续3次 | 未开始 | NOT_STARTED |  |  |  |
| B3 | 巡线交接 | 唯一控制权和黑线重捕获 | 未开始 | NOT_STARTED |  |  |  |
| B4 | 故障注入 | 所有已列故障安全停车 | 未开始 | NOT_STARTED |  |  |  |
