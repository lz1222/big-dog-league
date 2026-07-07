# RK Line System Scripts

This package contains operator scripts for the Go2 line-following system. The
scripts can be run from the source tree or from the installed package.

## System Chain

```text
RealSense D435i
-> /camera/color/image_raw
-> real_line_tracker_node
-> /perception/line_track
-> line_follower_node
-> /navigation/cmd_vel
-> cmd_vel_udp_forwarder.py or cmd_vel_bridge_node
-> Go2 Sport Move()
-> robot motion
```

The startup script launches the full chain, but it does not publish
`/mission/start`. Start line following only after the camera image and
`line_visible=true` have been confirmed.

## Environment

The actual robot runtime is ROS2 Foxy, normally under `~/rk_inspection_ws`.
The VM is a ROS2 Humble development and code-check environment. Humble-only
build issues in the VM do not imply the robot workspace is broken.

## Dependencies

Install system tools:

```bash
sudo apt install procps coreutils
```

Install image debug tools on the robot/Foxy runtime:

```bash
sudo apt install ros-foxy-rqt-image-view ros-foxy-image-view
```

On the VM/Humble development environment, use the matching `ros-humble-*`
packages instead.

The scripts default to `~/rk_inspection_ws`. If the workspace is somewhere
else, set:

```bash
export RK_INSPECTION_WS=/path/to/rk_inspection_ws
```

## Run Modes

Source mode:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/start_line_system.sh
```

Install mode:

```bash
~/rk_inspection_ws/install/rk_bringup/share/rk_bringup/scripts/start_line_system.sh
```

The other scripts support the same two locations.

## Commands

Start all line-system nodes without starting motion:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/start_line_system.sh
```

The startup script defaults to the verified SDK UDP bridge
and starts every process with `nohup` in the background. It does not use tmux,
so closing the SSH terminal will not stop the nodes.

Set line-following speeds before starting the script:

```bash
export RK_LINE_MIN_SPEED=0.27
export RK_LINE_BASE_SPEED=0.30
export RK_LINE_MID_SPEED=0.28
export RK_LINE_SLOW_SPEED=0.27
export RK_SHORT_LOST_LINEAR_SPEED=0.27
export RK_SEARCH_LINEAR_SPEED=0.27
export RK_START_ECONOMIC_GAIT=true
export RK_SDK_INTERFACE=eth0
~/rk_inspection_ws/src/rk_bringup/scripts/start_line_system.sh
```

`RK_LINE_MIN_SPEED` is the nonzero forward-speed floor. Keep it at `0.27` or
higher on Go2, because lower values may not produce a stable walking gait. The
startup script sets `bridge_max_linear_x` to `RK_LINE_BASE_SPEED` unless
`RK_GO2_BRIDGE_MAX_LINEAR_X` is set explicitly, so the bridge does not clip the
line follower below the requested speed.

For one-off launch testing on the robot/Foxy runtime, pass the same values as
launch arguments:

```bash
ros2 launch rk_bringup competition_line_nav.launch.py \
  start_economic_gait:=true \
  sdk_interface:=eth0 \
  debug:=false \
  enable_depth:=false \
  rgb_camera.profile:=424x240x15 \
  line_min_speed:=0.27 \
  line_base_speed:=0.30 \
  line_mid_speed:=0.28 \
  line_slow_speed:=0.27 \
  bridge_max_linear_x:=0.30
```

The line launch switches the robot to Unitree `economic_gait` by default before
mission start. To switch posture or gait manually while testing separate nodes,
stop line following first, run the SDK motion action, then restart line
following:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/mission_stop.sh
ros2 run rk_go2_sdk_bridge go2_sdk_motion_action eth0 balance_stand 1.0
ros2 run rk_go2_sdk_bridge go2_sdk_motion_action eth0 economic_gait 1.0
~/rk_inspection_ws/src/rk_bringup/scripts/mission_start.sh
```

The available SDK actions are `stand_up`, `balance_stand`, `economic_gait`,
`front_jump`, `recovery_stand`, and `stop_move`. The Python `gait_control_node`
still publishes `cmd_vel` for fixed actions; its body-height and recovery-stand
adapters are placeholders until the Unitree posture APIs are wired there.

VM/Humble development build note:

```bash
cd /home/lzbb/桌面/rk_inspection_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --build-base /tmp/rk_inspection_build \
  --install-base install
```

That `--build-base` workaround is only for VM/Humble builds from a non-ASCII
path. It is not a robot/Foxy runtime requirement.

The startup script writes logs and PID files here:

```bash
~/rk_line_logs/
~/rk_line_runtime/pids
```

After confirming `line_visible=true`, start line following once:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/mission_start.sh
```

With `continuous_search_enabled: true`, this mission start remains active until
`mission_stop.sh` is sent. If the line is briefly lost, navigation keeps
searching and resumes line following after the configured reacquire frames.

Stop line following without killing the background nodes:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/mission_stop.sh
```

Emergency stop and kill all related processes:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/stop_line_system.sh
```

Open image debug view from a VNC graphical desktop terminal:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/view_line_debug.sh
```

Check the ROS topic chain:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/check_line_system.sh
```

For installed scripts, replace `~/rk_inspection_ws/src/rk_bringup/scripts/`
with `~/rk_inspection_ws/install/rk_bringup/share/rk_bringup/scripts/`.

## Background Processes

`start_line_system.sh` starts these processes in the background:

- `sdk_server`: existing Go2 SDK UDP server.
- `cmd_vel_udp_forwarder`: `/navigation/cmd_vel` to SDK UDP.
- `realsense_camera`: low-bandwidth color stream.
- `real_line_tracker`: `/camera/color/image_raw` to `/perception/line_track`.
- `line_follower`: `/perception/line_track` to `/navigation/cmd_vel`.

Useful log commands:

```bash
tail -f ~/rk_line_logs/real_line_tracker.log
tail -f ~/rk_line_logs/line_follower.log
tail -f ~/rk_line_logs/cmd_vel_udp_forwarder.log
```

## Troubleshooting

`/navigation/cmd_vel` is always zero:

- Confirm `line_visible=true` in `~/rk_line_logs/real_line_tracker.log`.
- Confirm `/mission/start` was sent manually with `mission_start.sh`.
- Check `line_follower_node` logs in `~/rk_line_logs/line_follower.log`.

`line_visible=false`:

- Confirm `/camera/color/image_raw` has a camera publisher.
- Open `view_line_debug.sh` from VNC and inspect
  `/perception/debug/line_overlay` and `/perception/debug/line_mask`.
- Check lighting, black-line contrast, and camera alignment.

Bridge only receives zero velocity:

- Confirm `/navigation/cmd_vel` is nonzero in the `cmd_vel` tmux window.
- Run `check_line_system.sh` and confirm `/navigation/cmd_vel` has exactly one
  publisher, `line_follower_node`.
- Confirm normal ROS nodes use `ROS_DOMAIN_ID=10`.

`rqt_image_view` cannot see overlay:

- Run `view_line_debug.sh` in a VNC graphical desktop terminal, not pure SSH.
- The normal low-bandwidth line launch uses `debug:=false`, so overlay topics
  are disabled by default. Restart with `debug:=true` only when inspecting
  perception.
- Confirm `/perception/debug/line_overlay` appears in `ros2 topic list`.

Robot does not move but topics show velocity:

- Confirm `go2_sdk_udp_server` is running in the `bridge` window.
- Confirm UDP `127.0.0.1:15001` is reachable on the robot host.
- Confirm the robot is in a mode where `Unitree SportClient.Move()` commands
  are accepted.
