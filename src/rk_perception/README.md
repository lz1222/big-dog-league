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

`real_sign_detector_node` detects competition signs from an RGB stream and
publishes `rk_interfaces/msg/SignDetectionArray` on:

```bash
/perception/sign_detections
```

It tries QR/text labels first. For the three yellow triangular warning signs,
it finds the yellow sign area and then matches the inner black symbol template,
so `electric_shock`, `strong_oxidizer`, and `radiation` can be separated even
though their outer color is the same. HSV color rules are kept only as a
field-tunable fallback for older colored signs and platform markers.

For the Go2 front camera, platform markers use a separate safe path. A red
outer ring only locates a candidate; the inner black-and-white `place_1` or
`place_2` pattern is matched against the auditable templates in
`resources/place_marker_templates`. The detector searches only ±15 degrees
and publishes this path only after 5 matching frames in a 7-frame window with
no high-confidence conflict. Ambiguous or non-marker red objects produce no
platform detection.

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

For Go2 built-in front-camera warning-sign recognition, use the SDK-only
tools in `rk_go2_sdk_bridge` instead of ROS2 topics:

```bash
./install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_warning_sign_sdk_loop.sh \
  eth0 \
  --dry-run

./install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_warning_sign_sdk_loop.sh \
  eth0
```

Capture one Go2 front-camera image without ROS2:

```bash
./install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_capture_image \
  eth0 \
  /tmp/go2_front.jpg
```

Classify a saved image without ROS2:

```bash
python3 src/rk_go2_sdk_bridge/scripts/warning_sign_image_classifier.py \
  /tmp/go2_front.jpg
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
