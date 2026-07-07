# Grasp-with-the-Unitree-D1

Demonstrations of Using the Unitree D1 Robotic Arm to Open Cabinet Doors and Grasp Objects

[Click here to watch the video](https://www.bilibili.com/video/BV1kYCUYoE1Q)

Grasping Scissors (5x Speed)  
![demo-1](docs/demo-1.gif)

Grasping Phillips Screwdriver (Tool selected based on real-time audio recognition)  
![demo-2](docs/demo-2.gif)

Grasping Flathead Screwdriver  
![demo-3](docs/demo-3.gif)

Opening Drawer  
![demo-4](docs/demo-4.gif)

Usage Example: `src/test_grasp.py`

How to Build from Scratch?

1. Use the script `src/get_arm_joint_angle.cpp` to record key poses for grasping:
   1. After running it in the terminal, you can get real-time joint angles. On MAC, use the shortcut by pressing 'r' followed by 'ENTER' to record.
   2. Key poses will be saved in `build/servo_commands.txt`, which can be copied into multi-joint angle control programs, such as `left_1.cpp`.
   
2. Implement multi-joint angle control for grasping, split into multiple stages (e.g., opening the left cabinet door is split into `left_1.cpp` and `left_2.cpp`) because:
   1. It makes debugging easier;
   2. Increases the accuracy of grasping;
   3. Helps alleviate communication message delay (alternatively, add a sleep command or wait for the previous message to execute before sending the next one).
   
3. Ensure that the angle of the sixth joint remains unchanged after grasping the tool until it is released, to prevent it from dropping.

4. Modify `CMakeLists.txt`, use `cmake` to build, and `make` to compile into binary shared libraries, such as `libleft_1.so` and `libleft_2.so`.

5. Call the shared libraries in C++ or Python to implement grasping. Example usage can be found in `src/test_grasp.py`.

- Refer to the Unitree D1 Robotic Arm Development Guide: https://support.unitree.com/home/zh/developer/D1Arm_services

## Servo Heating Diagnosis

This workspace adds a passive diagnostic program:

```bash
cmake -S third_party/unitree_d1_sdk -B /tmp/unitree_d1_sdk_build
cmake --build /tmp/unitree_d1_sdk_build --target d1_servo_heat_diagnosis -j2
```

Run it on the robot while the D1 arm is powered, but do not send new motion
commands during the test:

```bash
source scripts/source_unitree_ros2.sh eth0
LD_LIBRARY_PATH=$PWD/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-} \
  /tmp/unitree_d1_sdk_build/d1_servo_heat_diagnosis eth0 30 1.0 2.0
```

Arguments:

- `eth0`: Unitree/D1 network interface.
- `30`: sampling duration in seconds.
- `1.0`: speed warning threshold in deg/s.
- `2.0`: angle range warning threshold in degrees.

The tool only subscribes to `current_servo_angle`, `arm_Feedback`, and
`rt/arm_Feedback`. It does not send `rt/arm_Command`.

If a servo is marked `CHECK`, it is still moving, drifting, or jittering while
powered, which can cause heat through repeated position correction. If all
servos are stable but a joint is still hot, suspect holding torque, a hard
limit, overextended pose, gripper clamping, calibration offset, or mechanical
friction.
