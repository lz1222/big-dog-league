# Stage-One Mock Race Checklist

Before running:

- Confirm ROS2 Humble is sourced.
- Confirm `colcon build --symlink-install` passes.
- Confirm `source install/setup.bash` is run in the current shell.
- Confirm `build/`, `install/`, and `log/` are not committed.

Mock acceptance checks:

- `ros2 launch rk_bringup mock_competition.launch.py` starts.
- Mock perception topics publish data.
- `/navigation/line_follower/state` reports the line follower FSM state.
- `/navigation/cmd_vel` publishes `Twist`.
- Mock locomotion and arm action servers respond.
- `/safety/estop` service exists.
- `/mission/run` reaches `DONE`.

Line navigation standalone check:

```bash
ros2 launch rk_bringup vision_nav_debug.launch.py \
  use_mock_perception:=true \
  auto_start:=false
ros2 run rk_tools line_nav_test_client_node --ros-args \
  -p duration_sec:=5.0
```

Manual mission check:

```bash
ros2 launch rk_bringup mock_competition.launch.py auto_start:=false
ros2 run rk_tools mission_client_node
```

Single-stage mission check:

```bash
ros2 launch rk_bringup mock_competition.launch.py \
  auto_start:=true \
  start_stage:=FOLLOW_TO_STAIRS \
  end_stage:=FOLLOW_TO_STAIRS \
  navigation_debug:=true
```
