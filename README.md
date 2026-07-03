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

To debug a single mission stage, pass the same stage name to
`start_stage` and `end_stage`:

```bash
ros2 launch rk_bringup mock_competition.launch.py \
  auto_start:=true \
  start_stage:=FOLLOW_TO_STAIRS \
  end_stage:=FOLLOW_TO_STAIRS \
  navigation_debug:=true
```

## Debug Line Navigation

Mock line tracker + line follower only:

```bash
source install/setup.bash
ros2 launch rk_bringup vision_nav_debug.launch.py \
  use_mock_perception:=true \
  auto_start:=true \
  debug_log:=true
```

Real line tracker + line follower:

```bash
ros2 launch rk_bringup vision_nav_debug.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  enable_debug_image:=true \
  debug_log:=true
```

Manual start/stop helper for line navigation:

```bash
ros2 run rk_tools line_nav_test_client_node --ros-args \
  -p duration_sec:=5.0
```

Useful topics while testing:

```bash
ros2 topic echo /perception/line_track
ros2 topic echo /navigation/line_follower/state
ros2 topic echo /navigation/cmd_vel
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
- `/navigation/line_follower/state`: `std_msgs/msg/String`
- `/locomotion/execute_motion`: `rk_interfaces/action/ExecuteMotion`
- `/arm/execute_task`: `rk_interfaces/action/ExecuteArmTask`
- `/mission/run`: `rk_interfaces/action/RunMission`
- `/safety/estop`: `std_srvs/srv/SetBool`

Do not edit generated files in `build/`, `install/`, or `log/`.
