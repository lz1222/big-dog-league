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
-> cmd_vel_udp_forwarder.py
-> UDP 127.0.0.1:15001
-> go2_sdk_udp_server
-> Unitree SportClient.Move()
-> robot motion
```

The startup script launches the full chain, but it does not publish
`/mission/start`. Start line following only after the camera image and
`line_visible=true` have been confirmed.

## Dependencies

Install system tools:

```bash
sudo apt install tmux procps coreutils
```

Install image debug tools:

```bash
sudo apt install ros-foxy-rqt-image-view ros-foxy-image-view
```

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
(`RK_GO2_BRIDGE_TYPE=sdk_udp`). Use `RK_GO2_BRIDGE_TYPE=unitree_driver` only
when intentionally testing the older `rk_unitree_driver` path.

Attach to the tmux session:

```bash
tmux attach -t rk_line
```

After confirming `line_visible=true`, start line following once:

```bash
~/rk_inspection_ws/src/rk_bringup/scripts/mission_start.sh
```

Stop line following without killing all windows:

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

## Tmux Windows

`start_line_system.sh` creates the `rk_line` tmux session with these windows:

- `bridge`: Go2 SDK UDP server and `/navigation/cmd_vel` UDP forwarder.
- `camera`: RealSense camera launch.
- `vision_nav`: line tracker and line follower debug launch.
- `line_track`: `/perception/line_track` echo.
- `cmd_vel`: `/navigation/cmd_vel` echo.
- `system_check`: live topic list watch.

## Troubleshooting

`/navigation/cmd_vel` is always zero:

- Confirm `line_visible=true` in the `line_track` tmux window.
- Confirm `/mission/start` was sent manually with `mission_start.sh`.
- Check `line_follower_node` logs in the `vision_nav` window.

`line_visible=false`:

- Confirm `/camera/color/image_raw` has a camera publisher.
- Open `view_line_debug.sh` from VNC and inspect
  `/perception/debug/line_overlay` and `/perception/debug/line_mask`.
- Check lighting, black-line contrast, and camera alignment.

Bridge only receives zero velocity:

- Confirm `/navigation/cmd_vel` is nonzero in the `cmd_vel` tmux window.
- Confirm `cmd_vel_udp_forwarder.py` is running in the `bridge` window.
- Confirm normal ROS nodes use `ROS_DOMAIN_ID=10`.

`rqt_image_view` cannot see overlay:

- Run `view_line_debug.sh` in a VNC graphical desktop terminal, not pure SSH.
- Confirm `vision_nav` was started with `enable_debug_image:=true`.
- Confirm `/perception/debug/line_overlay` appears in `ros2 topic list`.

Robot does not move but topics show velocity:

- Confirm `go2_sdk_udp_server` is running in the `bridge` window.
- Confirm UDP `127.0.0.1:15001` is reachable on the robot host.
- Confirm the robot is in a mode where `Unitree SportClient.Move()` commands
  are accepted.
