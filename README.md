# RK Inspection ROS2 Mock Workspace

This workspace contains the stage-one mock skeleton for the RK multimodal
inspection robot project. It targets ROS2 Humble and keeps all Unitree Go2,
Unitree D1 arm, and RealSense D435i integration points mocked.

## Build

```bash
cd ~/rk_inspection_ws
colcon build --symlink-install
source install/setup.bash
```

The helper script runs the same build flow and attempts rosdep first:

```bash
./scripts/build_all.sh
```

## Run The Mock Mission

```bash
source install/setup.bash
ros2 launch rk_bringup mock_competition.launch.py
```

The launch file starts mock perception, line following, mock locomotion, mock
arm control, safety service, and the mission state machine. The mission starts
automatically by default.

To disable automatic start and trigger manually:

```bash
ros2 launch rk_bringup mock_competition.launch.py auto_start:=false
ros2 run rk_tools mission_client_node
```

## Run The Two-Step Walk Test

This test publishes a conservative forward `Twist` command to
`/navigation/cmd_vel` for two seconds, then publishes zero velocity for one
second and exits.

```bash
source install/setup.bash
ros2 run rk_tools two_step_walk_test_node
```

To watch the velocity commands in another terminal:

```bash
ros2 topic echo /navigation/cmd_vel
```

For a slower or shorter hardware check:

```bash
ros2 run rk_tools two_step_walk_test_node --ros-args -p forward_speed:=0.08 -p walk_duration:=1.5
```

## Main Interfaces

- `/perception/line_track`: `rk_interfaces/msg/LineTrack`
- `/perception/sign_detections`: `rk_interfaces/msg/SignDetectionArray`
- `/perception/item_tags`: `rk_interfaces/msg/ItemTagArray`
- `/navigation/cmd_vel`: `geometry_msgs/msg/Twist`
- `/gait/control_lock`: `std_msgs/msg/Bool`
- `/locomotion/execute_motion`: `rk_interfaces/action/ExecuteMotion`
- `/arm/execute_task`: `rk_interfaces/action/ExecuteArmTask`
- `/mission/run`: `rk_interfaces/action/RunMission`
- `/safety/estop`: `std_srvs/srv/SetBool`

## Three-Person Development Split

See `docs/three_person_development_plan.md` for the file ownership map,
frozen ROS interfaces, merge rules, and staged integration checklist.

Do not edit generated files in `build/`, `install/`, or `log/`.
