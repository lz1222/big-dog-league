# RK Perception

This package provides perception publishers for the RK inspection robot.

## Real D435i Line Tracker

`real_line_tracker_node` detects a black line from an Intel RealSense D435i RGB
image and publishes the existing `rk_interfaces/msg/LineTrack` message on:

```bash
/perception/line_track
```

The node subscribes to the RGB image topic:

```bash
/camera/color/image_raw
```

## Real Sign Detector

`real_sign_detector_node` detects simple competition signs from the same RGB
stream and publishes `rk_interfaces/msg/SignDetectionArray` on:

```bash
/perception/sign_detections
```

It tries QR/text labels first and uses HSV color rules as a field-tunable
fallback. The default values map `place_1`, `place_2`, `electric_shock`,
`strong_oxidizer`, and `radiation` into the sign messages already consumed by
the mission code.

`realsense2_camera` is intentionally not a hard dependency of this package.
Install and start the RealSense ROS 2 wrapper separately:

```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

OpenCV for Python must also be available in the runtime environment. For
example:

```bash
sudo apt install python3-opencv ros-humble-cv-bridge
```

Run the real tracker:

```bash
ros2 run rk_perception real_line_tracker_node
```

Or switch between mock and real line tracking with launch:

```bash
ros2 launch rk_perception perception.launch.py use_mock_perception:=true
ros2 launch rk_perception perception.launch.py use_mock_perception:=false
ros2 launch rk_perception perception.launch.py use_mock_perception:=false debug_log:=true
```

If your RealSense launch publishes a different color image topic, override it:

```bash
ros2 launch rk_perception perception.launch.py \
  use_mock_perception:=false \
  image_topic:=/camera/camera/color/image_raw
```

Run the sign detector by itself:

```bash
ros2 run rk_perception real_sign_detector_node --ros-args \
  --params-file src/rk_perception/config/perception.yaml \
  -p image_topic:=/camera/color/image_raw
```

Run sign recognition plus conservative Go2 body actions:

```bash
ros2 launch rk_bringup sign_action_debug.launch.py \
  start_realsense:=true \
  start_gait_control:=true \
  dry_run_action:=false
```

Watch the detector and action mapping:

```bash
ros2 topic echo /perception/sign_detections
ros2 topic echo /sign_action/status
ros2 topic echo /gait/command_json
```

## OpenCV Pipeline

`real_line_tracker_node` uses this image processing flow:

1. Convert `sensor_msgs/Image` to a BGR OpenCV image with `cv_bridge`.
2. Crop the lower image region as the line tracking ROI.
3. Convert the ROI to grayscale.
4. Apply binary inverse thresholding so the black line becomes foreground.
5. Find contours and select the largest valid contour.
6. Compute the contour centroid for `lateral_error`.
7. Estimate `heading_error` with `cv2.fitLine`.
8. Publish confidence from the contour area ratio.

When no valid contour is found, the node publishes `line_visible=false` and
`confidence=0.0` while keeping the `/perception/line_track` topic and
`LineTrack` message unchanged.
