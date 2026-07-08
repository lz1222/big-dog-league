# rk_arm_control

Fixed-position arm task control for the Unitree Go2 inspection mission.

This package intentionally avoids vision feedback and inverse kinematics. It
executes hard-coded task sequences from `config/arm_poses.yaml`, publishes arm
state, and exposes the existing `/arm/execute_task` action used by
`rk_mission`. It also provides `/arm/command_json` for quick field testing.

## New Arm Adapter Framework

The original D1 arm is treated as unavailable for the replacement-arm path.
`new_arm_task_node` keeps the same upper-layer interfaces and moves all
hardware-specific logic behind an Adapter:

```text
new_arm_task_node
  -> DryRunArmAdapter
  -> SdkBridgeArmAdapter
       -> /arm/sdk_bridge/command_json
       -> external vendor SDK bridge
```

Main files:

- `config/new_arm_poses.yaml`: fixed joint poses and task sequences.
- `config/new_arm_params.yaml`: topics, perception confirmation, and adapter
  mode.
- `rk_arm_control/adapters/base.py`: common hardware interface.
- `rk_arm_control/adapters/dry_run_adapter.py`: safe no-hardware mode.
- `rk_arm_control/adapters/sdk_bridge_adapter.py`: JSON bridge for the new arm
  SDK.

The grasping plan is fixed-pose first. Camera detections on
`/perception/item_tags` or `/perception/object_xy_json` are used as optional
confirmation, not as a hard dependency, so the mission can still run if vision
is unstable.

Start the replacement-arm framework in dry-run mode:

```bash
ros2 launch rk_arm_control new_arm_task.launch.py
```

Send a fixed-pose task:

```bash
ros2 topic pub --once /arm/command_json std_msgs/msg/String \
  "{data: '{\"task\":\"PICK_START\"}'}"
```

Watch status and lock:

```bash
ros2 topic echo /arm/status
ros2 topic echo /arm/control_lock
```

Simulate a camera confirmation target:

```bash
ros2 topic pub --once /perception/object_xy_json std_msgs/msg/String \
  "{data: '{\"item_type\":\"start_item\",\"x\":0.12,\"y\":-0.03,\"confidence\":0.90}'}"
```

Switch to SDK bridge mode by editing `config/new_arm_params.yaml`:

```yaml
new_arm:
  adapter:
    mode: sdk_bridge
```

Then run a bridge listener for the real arm SDK and watch the outgoing command:

```bash
ros2 topic echo /arm/sdk_bridge/command_json
```

`SdkBridgeArmAdapter` has a `unitree_d1_json_reference` command format that
mirrors the imported Unitree D1 example style (`funcode=2` multi-joint JSON).
It is only a protocol reference; for a replacement arm, prefer a small
vendor-specific bridge process that consumes `generic_json`.

Current hardware mode: **DryRun**. No real arm topics or services such as
`/arm/joint_cmd` or `/gripper/cmd` were found in this repository. The real
driver should be connected inside `DryRunArmAdapter` in
`rk_arm_control/arm_task_node.py` at the TODO comments for joint and gripper
commands.

The camera-XY D1 pick flow requested for the fixed grasp platform is also kept
in this package as `d1_pick_node`. A separate `rk_d1_arm_control` package was
not added because this repository already has `rk_arm_control`, `/arm/status`,
and `/arm/control_lock`. Do not run `arm_task_node` and `d1_pick_node` at the
same time because both listen on `/arm/command_json`.

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

## D1 Camera-XY Pick Node

This node uses the latest camera XY target, applies a simple 2D affine
`camera_to_arm` transform, then uses fixed Z heights from
`config/d1_arm_params.yaml`.

Subscribed topics:

- `/arm/command_json` (`std_msgs/msg/String`)
- `/perception/object_xy_json` (`std_msgs/msg/String`)
- `/perception/object_xy` (`geometry_msgs/msg/PointStamped`)

Published topics:

- `/arm/status` (`std_msgs/msg/String`, JSON)
- `/arm/control_lock` (`std_msgs/msg/Bool`)

Supported commands:

- `{"task":"PICK_BY_CAMERA"}`
- `{"task":"PICK_START"}`
- `{"task":"PLACE_TRANSFER"}`
- `{"task":"HOME"}`
- `{"task":"OPEN_GRIPPER"}`
- `{"task":"CLOSE_GRIPPER"}`
- `{"task":"ABORT"}`

Start dry-run:

```bash
ros2 launch rk_arm_control d1_pick.launch.py dry_run:=true
```

Start real D1 SDK mode:

```bash
ros2 launch rk_arm_control d1_pick.launch.py dry_run:=false
```

Current real SDK status: **not fully wired**. The imported D1 SDK under
`third_party/unitree_d1_sdk` exposes C++ DDS examples that publish JSON strings
to `rt/arm_Command`, including:

- `third_party/unitree_d1_sdk/src/joint_angle_control.cpp`
- `third_party/unitree_d1_sdk/src/multiple_joint_angle_control.cpp`
- `third_party/unitree_d1_sdk/src/joint_enable_control.cpp`
- `third_party/unitree_d1_sdk/src/arm_zero_control.cpp`

No confirmed Python Cartesian XYZ API or safe gripper API was found. Therefore
all real hardware hooks are isolated in `UnitreeD1SdkAdapter` inside
`rk_arm_control/d1_pick_node.py`; with `dry_run:=false`, commands fail
gracefully with clear TODO logs instead of crashing. To finish real hardware
wiring, provide either a callable D1 Python/C++ binding for Cartesian pose and
gripper commands, or fill `pose_table.calibrated_points_json` with measured
fixed-platform XYZ-to-joint samples and wire the joint publisher in
`UnitreeD1SdkAdapter._send_joint_pose()`.

Simulate camera XY:

```bash
ros2 topic pub --once /perception/object_xy_json std_msgs/msg/String "{data: '{\"x\":0.12,\"y\":-0.03,\"confidence\":0.90,\"frame_id\":\"camera_link\",\"stamp\":1780000000.0}'}"
```

Execute pick:

```bash
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"PICK_BY_CAMERA\"}'}"
```

Place to transfer platform:

```bash
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"PLACE_TRANSFER\"}'}"
```

Home, gripper, and abort:

```bash
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"HOME\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"OPEN_GRIPPER\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"CLOSE_GRIPPER\"}'}"
ros2 topic pub --once /arm/command_json std_msgs/msg/String "{data: '{\"task\":\"ABORT\"}'}"
```

Watch state:

```bash
ros2 topic echo /arm/status
ros2 topic echo /arm/control_lock
```

`d1_pick_node` statuses include `IDLE`, `RUNNING`, `DONE`, `FAILED`,
`ABORTED`, `BUSY`, `TIMEOUT`, `NO_TARGET`, and `OUT_OF_WORKSPACE`.

State-machine integration:

- After reaching the grasp platform and stopping, publish
  `{"task":"PICK_BY_CAMERA"}`.
- Wait for `/arm/status` state `DONE`, then continue line following.
- If state is `NO_TARGET`, `OUT_OF_WORKSPACE`, `FAILED`, or `TIMEOUT`, retry
  once or enter manual handling.
- After reaching the transfer platform, publish `{"task":"PLACE_TRANSFER"}`.

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
