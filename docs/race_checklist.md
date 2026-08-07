# Stage-One Mock Race Checklist

Before running:

- Confirm ROS2 Humble is sourced.
- Confirm `colcon build --symlink-install` passes.
- Confirm `source install/setup.bash` is run in the current shell.
- Confirm `build/`, `install/`, and `log/` are not committed.

Mock acceptance checks:

- `ros2 launch rk_bringup mock_competition.launch.py` starts.
- Mock perception topics publish data.
- `/navigation/cmd_vel` publishes `Twist`.
- Mock locomotion and arm action servers respond.
- `/safety/estop` service exists.
- `/mission/run` reaches `DONE`.

Manual mission check:

```bash
ros2 launch rk_bringup mock_competition.launch.py auto_start:=false
ros2 run rk_tools mission_client_node
```
