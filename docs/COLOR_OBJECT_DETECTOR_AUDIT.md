# D435i Color RGB-D Audit

审计日期：2026-08-06。此记录仅描述新颜色目标定位模块，不改变已验收的抓取平台、警示标志和动作链路。

## 现状

1. D435i 启动入口为 `rk_bringup/launch/realsense_low_bandwidth.launch.py`，以 `camera` namespace 启动外部 `realsense2_camera/realsense2_camera_node`，默认开启 color/depth，profile 为 `640x480x15`。
2. 已有实际 RGB 输入由多个 launch 传入；当前低带宽 launch 未显式声明 RGB、aligned depth 和 CameraInfo 的最终 topic。仓库的已有深度功能默认使用 `/camera/camera/depth/image_rect_raw`，不是对齐深度。
3. 新节点的开发默认 topic 分别为 `/camera/camera/color/image_raw`、`/camera/camera/aligned_depth_to_color/image_raw` 和 `/camera/camera/color/camera_info`，全部是 YAML 参数，部署时必须以 `ros2 topic list -t` 的真机结果覆盖确认。
4. 已有感知配置的 frame 为 `d435i_color_optical_frame`，但静态扫描没有发现机械臂基座 frame 或 D435i 到机械臂的正式 TF 发布者。尝试在线 `ros2 topic list` / `tf2_tools view_frames` 未获得 ROS 图，环境连续返回 `std::bad_alloc`；因此不能将默认话题或 arm_base 视为真机已验收。
5. `rk_interfaces` 现有 `ItemTag` 含 `geometry_msgs/Pose`，但缺少颜色、确认状态、深度统计、相机坐标、机械臂坐标和失效语义，故新增 `ColorObjectDetection`，不复用该消息。
6. `rk_perception` 是 `ament_python` 包，已有 `config/`、`launch/`、`setup.py` console entry points；原有颜色检测、RGB-D 同步或物体三维定位代码均未找到。已有 `rk_tools/depth_wall_distance_node.py` 只做未对齐深度墙距抽样，不能复用为本模块。

## 冻结文件哈希

审计前后须一致的已验收文件：

| File | SHA-256 |
| --- | --- |
| `src/rk_perception/rk_perception/real_sign_detector_node.py` | `d0c1e7d14224ff52170d2b19cc7909ae958a4f3222f4739d9df788bec380133e` |
| `src/rk_perception/rk_perception/route_marker_detector_node.py` | `e637331acd47fe349e52b4743bd5a25dad7371739168feb63357304ebef0e85b` |
| `src/rk_perception/rk_perception/real_line_tracker_node.py` | `501f7d816787d0ddf343101bdce3e746294826f3c34147ef0fd991a96ef6999e` |
| `src/rk_mission/rk_mission/inspection_action_executor_node.py` | `e015d58bc31506c2b061bb84b4444546d97bd026ab9a399f73038c7d55a81dba` |

## 本次文件范围

新增：`rk_interfaces/msg/ColorObjectDetection.msg`、颜色检测 core/node、YAML、独立 launch、HSV 标定工具、core/node 测试和本审计记录。

修改：`rk_interfaces/CMakeLists.txt`、`rk_perception/package.xml`、`rk_perception/setup.py`。没有改动 `legacy/provincial_reference`、已验收标志识别、警示动作或任何机器狗/机械臂控制代码。

## 部署前核验

```bash
ros2 topic list -t | grep -E 'camera|aligned'
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo arm_base d435i_color_optical_frame
```

只有真机确认对齐深度、CameraInfo frame 和已标定 TF 均有效后，才可将 `grasp_ready=true` 作为视觉侧的准备条件；它不表示机械臂可执行抓取。
