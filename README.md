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

## Main Interfaces

- `/perception/line_track`: `rk_interfaces/msg/LineTrack`
- `/perception/sign_detections`: `rk_interfaces/msg/SignDetectionArray`
- `/perception/item_tags`: `rk_interfaces/msg/ItemTagArray`
- `/navigation/cmd_vel`: `geometry_msgs/msg/Twist`
- `/locomotion/execute_motion`: `rk_interfaces/action/ExecuteMotion`
- `/arm/execute_task`: `rk_interfaces/action/ExecuteArmTask`
- `/mission/run`: `rk_interfaces/action/RunMission`
- `/safety/estop`: `std_srvs/srv/SetBool`

Do not edit generated files in `build/`, `install/`, or `log/`.
