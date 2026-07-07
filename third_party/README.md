# Third-Party Dependencies

This directory is reserved for official upstream dependencies used by the RK
inspection workspace. Keep upstream code here instead of copying it into any
`rk_*` package.

## Expected Layout

```text
third_party/
  COLCON_IGNORE
  README.md
  unitree_ros2/
  unitree_sdk2/
  unitree_d1_sdk/
```

`COLCON_IGNORE` prevents the root RK workspace build from accidentally scanning
nested third-party workspaces.

## Unitree Sources

Use official Unitree repositories:

- `unitree_ros2`: https://github.com/unitreerobotics/unitree_ros2
- `unitree_sdk2`: https://github.com/unitreerobotics/unitree_sdk2
- `unitree_d1_sdk`: https://github.com/chen37058/Grasp-with-the-Unitree-D1
  (D1 arm examples; see `unitree_d1_sdk/SOURCE.md` for exact commit and local
  CMake adaptation)

Recommended tags for this integration stage:

- `unitree_ros2`: `v0.3.0`
- `unitree_sdk2`: `2.0.2`

The current acceptance scope is only ROS2 message and topic availability:
`unitree_api/msg/Request` and `/api/sport/request`. It does not validate real
robot motion.
