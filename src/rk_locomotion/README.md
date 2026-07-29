# rk_locomotion

Basic gait control layer for the first locomotion phase. It exposes a JSON
command topic and publishes velocity candidates to `/control/locomotion_cmd`.
The competition control mux owns the final `/navigation/cmd_vel` output.

## Interfaces

- Command input: `/gait/command_json` (`std_msgs/String`)
- Status: `/gait/status` (`std_msgs/String`)
- Control lock: `/gait/control_lock` (`std_msgs/Bool`)
- Debug: `/gait/debug` (`std_msgs/String`)
- Current mode: `/gait/current_mode` (`std_msgs/String`)
- Velocity candidate: `/control/locomotion_cmd` (`geometry_msgs/Twist`)

Supported commands:

- `STOP`
- `OBSTACLE_STOP`
- `RECOVERY_STAND`
- `HOLD_STABLE`
- `LOW_SPEED_MOVE`
- `TURN_IN_PLACE`
- `JUMP_START_OBSTACLE`
- `JUMP_END_OBSTACLE`
- `FRONT_JUMP_RECOVER`
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
`geometry_msgs/Twist` command to `/control/locomotion_cmd`. When
`/gait/control_lock` is true, `command_mux_node` may select that fresh command
and publish it to `/navigation/cmd_vel`.

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
  bridge_type:=sdk_udp \
  bridge_max_linear_x:=0.60 \
  bridge_max_angular_z:=1.00 \
  require_safety_data:=true
```

This standalone hardware-debug launch explicitly preserves the legacy direct
gait output and uses the same SDK UDP bridge as the working line-following stack:
`/navigation/cmd_vel -> cmd_vel_udp_forwarder.py -> go2_sdk_udp_server ->
Unitree SportClient.Move()`. If the SDK server is already running in another
terminal, add `start_sdk_server:=false`.

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

With the working SDK UDP bridge:

```bash
ros2 launch rk_go2_sdk_bridge go2_sdk_udp_bridge.launch.py
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

## Supervised FrontJump

The `start_jump` and `finish_jump` Action goals use one supervised SDK helper
implementation with independent `front_jump.start.*` and
`front_jump.finish.*` profiles. The supervisor owns the gait lock for the whole
Jump lifecycle, publishes only zero `Twist` candidates, confirms consecutive
new zero samples on `/navigation/cmd_vel`, and then invokes:

```text
go2_sdk_motion_action <network_interface> front_jump 0
```

The helper's return code 0 means only that the supervised software call was
accepted and its post-settle wait completed. It does not prove takeoff,
physical obstacle crossing, landing, or physical stability. The Action result
therefore reports `physical_crossing_unverified=true`.

The typed `/safety/estop_state` heartbeat is a fail-closed gate. A missing,
stale, or true heartbeat prevents the helper from starting; a true or stale
heartbeat while the helper or post-settle wait is active aborts local
supervision. Action cleanup never clears the system estop.

The development stale timeout is `0.20 s`. At a nominal 20 Hz heartbeat this
is four publish periods, so executor or transport delay approaching 200 ms
will intentionally produce a fail-closed abort. This margin must be measured
on the deployed computer; it is not a field-accepted timing value and must not
be relaxed merely to hide scheduling delays.

Likewise, consecutive zero messages on `/navigation/cmd_vel` prove only that
the final software `Twist` is zero. They do not prove that the robot is
physically stationary. The current `is_robot_stable()` placeholder is not used
as evidence of a successful landing or crossing.

The profile values in `config/gait_params.yaml`, including the default `eth0`
network interface, are development initial values rather than field-accepted
settings. A real FrontJump may only be tested in an isolated site with the
required physical safety controls. This PR's automated tests use fake clocks,
publishers, state inputs, and process doubles for the supervised flow, plus
temporary harmless Python subprocesses for process-group cleanup. They never
run the SDK helper or `SportClient.FrontJump()`.

### Lock and motion ownership

All non-STOP Action and JSON commands share one motion execution slot. The
slot records its entry type, motion name, reservation token, Action UUID or
JSON command identity, and lifecycle state. Action and JSON commands therefore
cannot bypass each other. STOP does not acquire the slot and may interrupt its
current owner. A rejected command does not change `/gait/status`.

FrontJump is the only owner of the gait-lock lifecycle while a Jump is active.
The lock publisher serializes transitions and periodic republishes. A
successful `publish()` only establishes
`lock_acquire_command_published=true` or
`lock_release_command_published=true`; it does not mean that
`command_mux_node` acknowledged the state. There is currently no typed mux
lock ACK. Before starting the helper, the available software evidence is only:

1. the lock=true message was successfully handed to the ROS publisher;
2. consecutive new, fresh, finite three-axis zero samples were observed on
   `/navigation/cmd_vel`;
3. the typed estop heartbeat was received, fresh, and false.

Any lock publish exception is latched fail-closed. An acquire failure prevents
the helper from starting. A release failure restores the internal desired
state to true, leaves cleanup incomplete, retains the persistent guard, and
rejects every later non-STOP motion. A later periodic republish never converts
the failed lifecycle transition into a successful acquire or release.

### Persistent cleanup guard and recovery

Before `Popen`, the node atomically writes a mode-0600 DIRTY guard under the
mode-0700 runtime directory:

```text
~/rk_line_runtime/front_jump_cleanup_guard.json
```

It contains a unique `cleanup_fault_id`, boot ID, reservation identity, helper
PID/PGID/session/start ticks/executable identity, lock publish evidence,
process-group cleanup evidence, and typed fault records. The helper is launched
in a new session. Cancel, STOP, estop, timeout, shutdown, and execution errors
terminate the complete PGID with SIGTERM and then SIGKILL if necessary, reap
the leader, and verify that the group is empty. Cleanup is not complete until
the whole group is absent, final zero was published, lock=false was published,
and the guard was safely finalized.

Every guard read or update revalidates the parent and target with `lstat`/
directory file descriptors: the parent must be owned by the effective user and
mode 0700, and an existing target must be an effective-user-owned regular
mode-0600 file, never a symlink. The replacement rechecks that evidence just
before `os.replace()`. If post-`Popen` identity capture cannot establish the
leader PID, PGID, session, start ticks, and executable identity within its
bounded deadline, the implementation never sends an unverified `killpg()`.
It may only best-effort signal and reap the `Popen` leader; it retains the
DIRTY guard and desired lock=true because whole-group cleanup is unproven.

A normal `rk_locomotion` node restart in the same compute boot never clears a
DIRTY guard or publishes a normal unlock. A complete compute reboot is
distinguished by Linux boot ID, but still keeps lock=true until live startup
checks receive a fresh false typed estop and a new consecutive final-cmd zero
window. Corrupt guards, unknown schemas, bad permissions, symbolic links, and
unverifiable process identities remain fail-closed.

An operator may recover a cleanup-only fault with the exact current ID:

```bash
ros2 topic pub --once /gait/command_json std_msgs/msg/String \
  "{data: '{\"command\":\"FRONT_JUMP_RECOVER\",\
\"cleanup_fault_id\":\"<current-id>\",\
\"confirm_no_front_jump_helper\":true}'}"
```

Recovery requires no active slot or worker, no active or identity-matching
helper group, fresh false typed estop, a new final-cmd zero window, a valid
guard, and no lock-publish, Action reservation, Action terminal-delivery, or
fatal-shutdown fault. The ID must exactly match. Old or repeated messages
cannot clear a newer fault. This command does not clear lock, Action, or fatal
shutdown faults, and it never claims that a physical FrontJump was undone.

### Shutdown and test isolation

`context.on_shutdown(node.request_shutdown_from_context)` is a non-blocking
backstop that touches only local Events/Conditions and never calls
`context.ok()` or another ROS API while the Context lock is held. Shutdown
first rejects new motion and signals the active slot, then the main loop
continues bounded `spin_once()` draining while the context remains valid. It
uses the following order:

```text
request_shutdown -> drain -> prepare_finalize_shutdown
                 -> executor.shutdown -> commit_finalize_shutdown
                 -> remove/destroy node -> context.try_shutdown
```

`prepare_finalize_shutdown()` keeps desired lock=true and never publishes an
unlock. Only `commit_finalize_shutdown(True)` may publish lock=false, and only
after executor shutdown succeeded, the guard/cleanup is clean, no safety fault
is latched, and ROS output remains valid. A failed executor shutdown, invalid
Context, or dirty/faulted cleanup stays fail-closed without a transient false
lock publication. If the context is already invalid, ROS output is disabled;
the local monotonic/poll/killpg/wait cleanup path still terminates, kills if
needed, and reaps the helper group. An undeliverable Action terminal state is
recorded as a separate fault.

Action and JSON slots use the common lifecycle
`RESERVED -> ACCEPTED -> EXECUTING -> STOPPING? -> FINALIZING -> DONE`.
`completion_event` is set only after worker exit, helper cleanup or an explicit
latched fault, the single terminal-call attempt, and internal reference cleanup.
Terminal delivery errors leave Action results unsuccessful and retain the
motion safety gate; no alternative terminal API call is attempted.

The Action name and every topic used by these tests are parameters, so tests
override their absolute defaults with a unique prefix. `GaitControlNode` also
accepts optional `front_jump_supervisor_factory`, `process_runner`,
`network_interface_validator`, and `executable_resolver` dependencies. Normal
startup uses the production defaults. Integration tests inject harmless
process doubles; a separate subprocess test launches only temporary Python
leader/child processes to verify SIGTERM/SIGKILL and reap behavior.

For a non-moving mux-integrated observation, watch:

```bash
ros2 topic echo /gait/control_lock std_msgs/msg/Bool
ros2 topic echo /control/locomotion_cmd geometry_msgs/msg/Twist
ros2 topic echo /navigation/cmd_vel geometry_msgs/msg/Twist
ros2 topic echo /safety/estop_state std_msgs/msg/Bool
```
