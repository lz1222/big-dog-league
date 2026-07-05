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
- `OBSTACLE_STOP`
- `RECOVERY_STAND`
- `HOLD_STABLE`
- `LOW_SPEED_MOVE`
- `TURN_IN_PLACE`
- `JUMP_START_OBSTACLE`
- `JUMP_END_OBSTACLE`
- `PRACTICAL_OBSTACLE_ZONE`
- `ENTER_OBSTACLE_ZONE`
- `OBSTACLE_FORWARD_SLOW`
- `OBSTACLE_TURN_LEFT`
- `OBSTACLE_TURN_RIGHT`
- `OBSTACLE_SIDE_ADJUST`
- `EXIT_OBSTACLE_ZONE`
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

## Practical Obstacle Zone

`PRACTICAL_OBSTACLE_ZONE` is the competition obstacle-area routine. It runs a
configurable low-speed step sequence from `config/gait_params.yaml` while
checking D435i depth and optional `LaserScan` safety data every control cycle.
The robot sends repeated zero velocity and fails the command when the front
clearance is below `obstacle_safety.front_stop_distance_m`, a side clearance is
below `obstacle_safety.side_stop_distance_m`, fresh sensor data is required but
missing, or the action is canceled.

The default obstacle route assumes the robot starts at the right-bottom entry
facing upward into the maze: go up the right corridor, turn left across the top,
turn left down the middle corridor, turn right across the bottom, turn right up
the left corridor, then turn left and leave from the upper-left exit.

For each maze step, tune the arrays under `practical_obstacle` in
`config/gait_params.yaml`:

- `step_distance_m`: forward distance in meters. Fill `0.0` for turn rows.
- `step_turn_angle_deg`: turn angle in degrees. Fill `0.0` for forward rows.
- `step_linear_speed`: forward speed in meters per second.
- `step_turn_speed_deg`: turning speed in degrees per second.
- `step_front_target_m`: optional front-wall target distance in meters. Use
  `0.0` to disable it for a row.

The node computes the actual runtime from those values:

```text
forward duration = step_distance_m / step_linear_speed
turn duration    = step_turn_angle_deg / step_turn_speed_deg
```

For repeatable entry positioning, set `entry_up` with a generous
`step_distance_m` and a front-wall target such as `step_front_target_m=0.50`.
The robot then drives forward until the top wall is about 0.50 m away, so
`turn_left_to_top` starts from a consistent pose even if the robot entered a
little early or late.

Launch the single-area test stack:

```bash
ros2 launch rk_bringup obstacle_practical.launch.py \
  backend:=mock \
  require_safety_data:=true
```

Switch `backend:=unitree_ros2` only after the mock run shows sane
`/navigation/cmd_vel` and `/gait/debug` output.

Run the full practical sequence through the action API:

```bash
ros2 action send_goal /locomotion/execute_motion \
  rk_interfaces/action/ExecuteMotion "{motion_name: avoid_zone}" \
  --feedback
```

Run the open-loop movement smoke test. This ignores depth safety and only
checks whether the robot can visibly move, so use it in open space:

```bash
ros2 action send_goal /locomotion/execute_motion \
  rk_interfaces/action/ExecuteMotion "{motion_name: open_loop_obstacle_test}" \
  --feedback
```

Run one primitive at a time:

```bash
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"ENTER_OBSTACLE_ZONE\"}'}"

ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"OBSTACLE_FORWARD_SLOW\",\"duration_sec\":0.5}'}"

ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"OBSTACLE_TURN_LEFT\",\"duration_sec\":0.4}'}"
```

Emergency stop:

```bash
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"OBSTACLE_STOP\"}'}"
```

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

## Stage 2 Obstacle Tests

The obstacle commands are conservative framework sequences, not aggressive
jumps. They stop first, run two short low-speed forward phases, and recover with
zero velocity. Keep watching `/navigation/cmd_vel` and `/gait/control_lock`
during bench testing.

Watch the control lock:

```bash
ros2 topic echo /gait/control_lock std_msgs/msg/Bool
```

Watch the velocity sent to the existing bridge:

```bash
ros2 topic echo /navigation/cmd_vel geometry_msgs/msg/Twist
```

Test the start obstacle sequence:

```bash
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"JUMP_START_OBSTACLE\"}'}"
```

Test the end obstacle sequence:

```bash
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"JUMP_END_OBSTACLE\"}'}"
```

Test STOP interrupting the start obstacle sequence:

```bash
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"JUMP_START_OBSTACLE\"}'}" &
sleep 1
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"STOP\"}'}"
```
