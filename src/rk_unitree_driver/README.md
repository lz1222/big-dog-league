# rk_unitree_driver

`rk_unitree_driver` bridges RK navigation `cmd_vel` commands to a Unitree Go2
Sport control backend.

## Backends

- `backend:=mock` is the default. It does not import `unitree_api`, does not
  publish Unitree requests, and only logs `Move` and `StopMove` commands. Use
  this mode for development or CI on machines without Unitree official ROS2
  packages.
- `backend:=unitree_ros2` is for a real Unitree ROS2 environment. It imports
  `unitree_api.msg.Request` and publishes Sport requests to
  `/api/sport/request` by default. Source the Unitree ROS2 workspace before
  using this mode.

## Commands

Mock mode:

```bash
ros2 run rk_unitree_driver cmd_vel_bridge_node --ros-args -p backend:=mock
```

Launch file, also mock by default:

```bash
ros2 launch rk_unitree_driver go2_cmd_vel_bridge.launch.py
```

Real robot mode must be selected explicitly:

```bash
ros2 launch rk_unitree_driver go2_cmd_vel_bridge.launch.py backend:=unitree_ros2
```

## Safety Notes

The bridge sends `StopMove` when it receives NaN or Inf values, commands above
the configured speed limits, a command timeout, or a shutdown signal such as
Ctrl+C. Keep `max_linear_x`, `max_angular_z`, and `cmd_timeout_sec` conservative
when using `backend:=unitree_ros2` on hardware.
