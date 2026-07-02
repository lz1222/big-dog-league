# rk_locomotion

Basic gait control layer for the first locomotion phase. It exposes a single
JSON command topic while reusing the existing `/navigation/cmd_vel` to Unitree
bridge.

## Interfaces

- Command input: `/gait/command_json` (`std_msgs/String`)
- Status: `/gait/status` (`std_msgs/String`)
- Control lock: `/gait/control_lock` (`std_msgs/Bool`)
- Debug: `/gait/debug` (`std_msgs/String`)
- Current mode: `/gait/current_mode` (`std_msgs/String`)

Supported commands:

- `STOP`
- `RECOVERY_STAND`
- `HOLD_STABLE`
- `LOW_SPEED_MOVE`
- `TURN_IN_PLACE`
- `BODY_HEIGHT_ADJUST`
- `SPEED_LIMIT`

`RECOVERY_STAND` and `BODY_HEIGHT_ADJUST` are intentionally left as adapter
TODOs until a stable Unitree low-level API is available in this workspace.
They return `FAILED` with a warning instead of modifying existing driver code.

## Velocity Mapping

`LOW_SPEED_MOVE` accepts `vx`, `vy`, and `wz` fields in the JSON command. The
gait layer validates and clamps all three values before publishing a
`geometry_msgs/Twist` command to `/navigation/cmd_vel`.

The current lower-level cmd_vel bridge has only been confirmed to map
`linear.x` and `angular.z` into Unitree Sport Move. `linear.y` / `vy` lateral
motion is kept in the gait interface for future obstacle-area alignment, but it
is not yet wired through the Unitree Sport Move adapter in this repository. If a
later phase needs true side-step motion, the lower-level Unitree adapter must be
extended first.

## Run

```bash
ros2 launch rk_locomotion gait_control.launch.py
```

With the existing Unitree bridge:

```bash
ros2 launch rk_unitree_driver go2_cmd_vel_bridge.launch.py
ros2 launch rk_locomotion gait_control.launch.py
```

## Test

```bash
ros2 run rk_locomotion gait_basic_test_node
```

The test calls `STOP`, `RECOVERY_STAND`, `HOLD_STABLE` for 3 seconds, and
`LOW_SPEED_MOVE` at `vx=0.1` for 2 seconds.

JSON example:

```bash
ros2 topic pub --once /gait/command_json std_msgs/String \
  "{data: '{\"command\":\"LOW_SPEED_MOVE\",\"vx\":0.1,\"duration_sec\":2.0}'}"
```
