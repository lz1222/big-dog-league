# B2.1-A Maze Handoff Summary — 2026-08-04

## Git Discrepancies vs Handoff Audit Snapshot

| Item | Expected (Handoff Prompt) | Actual |
|---|---|---|
| Branch | `master` | `避障` |
| HEAD | `c220f11a92d3eba7be479ab667e7044f4f30edb7` | `cb7aa7afe15e106c96fff556b9cf44b55a27d6f6` |
| origin/master | `c220f11` | Not on master branch; `避障` at cb7aa7a |
| Audit workspace | `/home/lqsdaba/daihao1/big-dog-league` | `/home/unitree/big-dog-league` |
| Working tree | Clean (only handoff docs untracked) | 34 files changed, +10051/-258 lines |
| CLAUDE_CODE_MAZE_HANDOFF.md | Present | **NOT FOUND** |
| `/tmp/claude_handoff_round15_replay_current.json` | Present | **NOT FOUND** |
| `evidence/` directory | Present | **NOT FOUND** at `/home/unitree/big-dog-league/evidence/` |

## Verified Items (This Session)

- Round15 rosbag SHA256: `6788e8687343471cfae44d008a316ad271b2677ce233e5c303103e85fa7b3b19` — MATCHES
- Round15 v2 geometry summary: gate_status=FAIL, motion_output=False, publisher_count=0 — MATCHES handoff claims
- 98/98 unit tests pass (Python 3.8.10)
- All scripts compile successfully
- motion_output=False confirmed in 365+117 dry_run_status frames across 2 static bags
- execution_allowed=False hardcoded (line 2387 of maze_first_turn_core.py)
- Zero references to cmd_vel, Twist, /api/sport, or SportClient in maze_first_turn_dry_run.py
- Dry run node publishes only on `/maze/first_turn/dry_run_status` (std_msgs/msg/String)
- Map build: ~14.7Hz (target ≥10Hz), median 4.2-10.2ms
- Body filter active: body_x=[-0.40, 0.40], body_y=[-0.18, 0.18]
- front_leg_self_filter_enabled: false (by design)
- Rear coverage: all three rear sectors insufficient → REVERSE_SHORT correctly blocked
- Front plate and side walls visible in maze entry position (2 wall_segments)
- Right side wall_endpoint causes static footprint collision at maze entry (matches Round15 v2 pattern)

## Missing/Outstanding

1. `docs/CLAUDE_CODE_MAZE_HANDOFF.md` — does not exist in either workspace
2. `/tmp/claude_handoff_round15_replay_current.json` — does not exist
3. `evidence/` directory — does not exist at expected path
4. `leg_self_filtered_points` — not in JSON output (front_leg_self_filter_enabled=false)
5. watchdog/estop Bool topics — not connected (both None in all frames)
6. Right side wall_endpoint static collision — body filter audit needed
7. Left side obstacle wall_segment association — still null in v2 replay

## Next Required Task

Audit right side wall_endpoint static collision root cause:
- Extract raw point cloud from `b2_1_a_static_maze_entry_retry_20260803_170707`
- Verify whether wall_000 starting point is Go2 self-structure echo
- If self-echo: adjust body_y_min_m or add right-leg local filter
- Target: current_footprint_safety.collision=false at maze entry

## Safety Confirmation

No nonzero motion commands were issued during this session. All verification was read-only
(unit tests, bag replay, code audit). No Twist, no SportClient, no /api/sport/request,
no go2_sdk_udp_bridge launch, no ARM, no gait change.
