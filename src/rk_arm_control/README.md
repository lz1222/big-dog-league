# rk_arm_control

Fixed-position arm task control for the Unitree Go2 inspection mission.

This package intentionally avoids vision feedback and inverse kinematics. It
executes hard-coded task sequences from `config/arm_poses.yaml`, publishes arm
state, and exposes the existing `/arm/execute_task` action used by
`rk_mission`. It also provides `/arm/command_json` for quick field testing.

Current hardware mode: **DryRun**. No real arm topics or services such as
`/arm/joint_cmd` or `/gripper/cmd` were found in this repository. The real
driver should be connected inside `DryRunArmAdapter` in
`rk_arm_control/arm_task_node.py` at the TODO comments for joint and gripper
commands.

## Build

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select rk_arm_control --symlink-install
source install/setup.bash
```

If `rk_interfaces` has not been built yet, build it together with this package:

```bash
colcon build --packages-select rk_interfaces rk_arm_control --symlink-install
source install/setup.bash
```

## Start

```bash
ros2 launch rk_arm_control arm_task.launch.py
```

Optional custom pose file:

```bash
ros2 launch rk_arm_control arm_task.launch.py config_file:=/path/to/arm_poses.yaml
```

## Topic Test Commands

```bash
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"HOME\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"OPEN_GRIPPER\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"CLOSE_GRIPPER\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"PICK_START\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"PLACE_TRANSFER\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"PICK_FIELD\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"PLACE_TARGET\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"ABORT\"}'}"
```

## Watch State

```bash
ros2 topic echo /arm/status
ros2 topic echo /arm/control_lock
```

Status is published as JSON:

```json
{
  "task": "PICK_START",
  "state": "RUNNING",
  "step": "MOVE_PICK_START",
  "success": true,
  "message": "step started"
}
```

States include `IDLE`, `RUNNING`, `DONE`, `FAILED`, `ABORTED`, `BUSY`, and
`TIMEOUT`.

## Mission Integration

The mission state machine can keep using the existing
`rk_interfaces/action/ExecuteArmTask` action on `/arm/execute_task`.

Suggested flow:

- After reaching the pick platform, send `PICK_START`.
- Wait for `/arm/status` state `DONE`, then continue line following.
- After reaching the transfer platform, send `PLACE_TRANSFER`.
- After reaching the field item platform, send `PICK_FIELD`.
- After reaching the target platform, send `PLACE_TARGET`.
- If state is `FAILED`, `TIMEOUT`, or `ABORTED`, enter manual handling or retry.

For compatibility with the current `rk_mission` code, these legacy action
task names are also accepted:

- `pick_start_item` -> `PICK_START`
- `drop_start_item` -> `PLACE_TRANSFER`
- `pick_field_item` -> `PICK_FIELD`
- `place_field_item` -> `PLACE_TARGET`

## Task Sequences

`PICK_START`:

`HOME -> OPEN_GRIPPER -> MOVE_PRE_PICK_START -> MOVE_PICK_START -> CLOSE_GRIPPER -> WAIT -> MOVE_LIFT_START -> HOME`

`PLACE_TRANSFER`:

`HOME -> MOVE_PRE_PLACE_TRANSFER -> MOVE_PLACE_TRANSFER -> OPEN_GRIPPER -> WAIT -> MOVE_LIFT_TRANSFER -> HOME`

`PICK_FIELD`:

`HOME -> OPEN_GRIPPER -> MOVE_PRE_PICK_FIELD -> MOVE_PICK_FIELD -> CLOSE_GRIPPER -> WAIT -> MOVE_LIFT_FIELD -> HOME`

`PLACE_TARGET`:

`HOME -> MOVE_PRE_PLACE_TARGET -> MOVE_PLACE_TARGET -> OPEN_GRIPPER -> WAIT -> MOVE_LIFT_TARGET -> HOME`

