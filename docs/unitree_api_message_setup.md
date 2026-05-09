# Unitree API Message/Topic Setup

This stage integrates the official Unitree ROS2 message packages as
third-party code so `rk_unitree_driver backend:=unitree_ros2` can create a
publisher on `/api/sport/request`.

This is not a real robot motion validation. Do not treat these checks as
`lowcmd`, MPC, gait planner, nav2, SLAM, or hardware movement verification.

## Directory Layout

```text
third_party/
  COLCON_IGNORE
  README.md
  unitree_ros2/
    cyclonedds_ws/
      src/
        cyclonedds.xml
        unitree/
          unitree_api/
          unitree_go/
          unitree_hg/
  unitree_sdk2/
```

Official code stays under `third_party/`. Do not copy Unitree sources into
`src/rk_unitree_driver` or any other `rk_*` package.

## Clone Official Sources

```bash
cd ~/rk_inspection_ws
mkdir -p third_party
git clone --branch v0.3.0 --depth 1 https://github.com/unitreerobotics/unitree_ros2.git third_party/unitree_ros2
git clone --branch 2.0.2 --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git third_party/unitree_sdk2
```

If a directory already exists, inspect it before replacing it:

```bash
git -C third_party/unitree_ros2 status --short
git -C third_party/unitree_sdk2 status --short
```

## Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  git cmake g++ build-essential \
  libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl
```

## Build SDK2

```bash
cd ~/rk_inspection_ws/third_party/unitree_sdk2
cmake -S . -B build -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$PWD/install
cmake --install build
```

## Build Unitree ROS2 Messages

```bash
source /opt/ros/humble/setup.bash
cd ~/rk_inspection_ws/third_party/unitree_ros2/cyclonedds_ws
colcon build --symlink-install
```

## Build RK Workspace

```bash
cd ~/rk_inspection_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

The root workspace contains `third_party/COLCON_IGNORE`, so this build keeps
the RK packages separate from the official Unitree workspace.

## Runtime Source Order

Use this order for message/topic acceptance:

```bash
source /opt/ros/humble/setup.bash
source ~/rk_inspection_ws/third_party/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/rk_inspection_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The helper script applies the same order and configures CycloneDDS:

```bash
source ~/rk_inspection_ws/scripts/source_unitree_ros2.sh lo
```

Use `lo` for local message/topic acceptance. Use a real Unitree-facing network
interface only when preparing for robot network tests.

## CycloneDDS And Network Checks

List local interfaces:

```bash
ip -brief address
```

For local message/topic acceptance:

```bash
source ~/rk_inspection_ws/scripts/source_unitree_ros2.sh lo
```

For a later robot network check, replace `<unitree_iface>` and `<robot-ip>`
with the actual interface and robot address:

```bash
sudo ip addr add 192.168.123.99/24 dev <unitree_iface>
sudo ip link set <unitree_iface> up
source ~/rk_inspection_ws/scripts/source_unitree_ros2.sh <unitree_iface>
./scripts/check_network.sh <robot-ip>
```

Do not count network reachability as motion verification.

## Acceptance Commands

Mock system remains the baseline:

```bash
source ~/rk_inspection_ws/install/setup.bash
ros2 launch rk_bringup mock_competition.launch.py
```

Unitree API message is available:

```bash
source ~/rk_inspection_ws/scripts/source_unitree_ros2.sh lo
ros2 interface show unitree_api/msg/Request
```

`rk_unitree_driver` creates `/api/sport/request`:

```bash
source ~/rk_inspection_ws/scripts/source_unitree_ros2.sh lo
ros2 run rk_unitree_driver cmd_vel_bridge_node --ros-args -p backend:=unitree_ros2
```

In another terminal with the same source command:

```bash
ros2 topic list | grep /api/sport/request
```

Passing this stage means:

- `ModuleNotFoundError: unitree_api` no longer appears.
- `ros2 interface show unitree_api/msg/Request` works.
- `ros2 topic list` shows `/api/sport/request`.
- `ros2 launch rk_bringup mock_competition.launch.py` still works.
- No real robot movement has been validated.
