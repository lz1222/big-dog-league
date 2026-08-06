# D435i 颜色与二维轮廓真机调试记录

## 范围和限制

本记录对应 D435i 真机调试，使用 `maze-fusion-t0-t3` 分支当前代码。
`shape` 表示 **2D projected contour shape**，不能称为三维物体分类，也不支持
球体、圆柱体、长方体或三棱锥的三维类别结论。本轮没有进行机械臂抓取。

当前仅支持限定场景下的 `circle` 与 `non-circle` 区分。圆柱和长方体在当前
侧视二维轮廓下均可表现为 `rectangle`，不能据此区分二者。

## 相机和 RGB-D 环境

- D435i 序列号：`247122070614`
- USB：3.2
- 固件：`5.17.3.10`
- 实际流配置：彩色和深度均为 640x480@15fps，`align_depth.enable=true`
- 彩色 topic：`/camera/color/image_raw`，编码 `rgb8`
- 对齐深度 topic：`/camera/aligned_depth_to_color/image_raw`，编码 `16UC1`
- CameraInfo topic：`/camera/color/camera_info`
- 相机光学 frame：`camera_color_optical_frame`
- CameraInfo K：`fx=606.7632446`、`fy=606.6812134`、
  `cx=317.0541992`、`cy=247.7298584`

正式检测 YAML 的三路输入已经与上述实际 RealSense topic 对齐；未启动底盘、
机械臂、任务状态机或抓取控制。

## HSV 候选标定

固定测试物表面主 Hue 实测为 90--91。该值落在原始 green（35--85）与 blue
（95--130）区间之间，因此仅在独立候选配置中将 green Hue 上限调整为 94：

`artifacts/color_shape_validation/hsv_candidate.yaml`

此候选文件用于本轮真机调试，正式比赛 HSV 参数保持原值。候选参数仍为
**DEVELOPMENT CANDIDATE / NOT FULLY FIELD VALIDATED**；在完成亮光、阴影、
相似背景等负样本测试前不得提升为正式比赛参数。

`expected_color=none` 时消息保持 `detected=false`、`confirmed=false`、
`grasp_ready=false`。设置 `expected_color=green` 后，候选参数可使固定颜色
目标被检测并确认。

## 限定场景轮廓观察

| 测试物与姿态 | 检测消息中的二维轮廓 | 说明 |
| --- | --- | --- |
| 球，近距离正视 | `circle` | 深度约 0.529m，已确认。 |
| 长方体，正视面 | `rectangle` | 4 顶点，已确认。 |
| 正三棱锥，两次摆放 | `unknown/5 vertices`；`square/4 vertices` | 两次均为 non-circle；未获得 `triangle` 消息。 |
| 直圆柱体，后续夹取所需正放侧视 | `rectangle` | 4 顶点，已确认；未将端面转向相机。 |

上述结果只支持球测试物的圆形投影与正三棱锥测试物 non-circle 投影的限定区分，
不构成三维物体分类结论。

## 坐标、TF 与安全状态

检测消息使用 `position_camera`，单位为米。示例球结果为
`(0.021, -0.092, 0.529)`，正放圆柱侧视示例为
`(0.019, -0.091, 0.523)`。

当前没有 `arm_base` TF。节点因而始终发布 `grasp_ready=false` 与
`reason=tf_unavailable`，未伪造 arm-base 坐标，也未启动任何机械臂抓取。

## 调试图兼容性修复

本机 OpenCV 5.0.0 与 ROS Foxy `cv_bridge` 的类型表缺少 `bgr8` 对应的
CV 类型 16，调用 `cv2_to_imgmsg(..., encoding='bgr8')` 会触发 `KeyError:16`。
检测节点现显式封装 uint8 `bgr8` 和 `mono8` 图像消息，保留输入图像 header，
避免该版本兼容性问题；已添加对应回归测试。

## 未完成验证

- 未进行每个距离 100 帧统计和深度真值误差统计。
- 未进行 10 分钟稳定性、CPU 或内存增长验证。
- 未验证 optical frame 三轴方向。
- ROS 图查询期间重复出现 `std::bad_alloc` 文本，尚未完成根因排查。
- 未完成亮光、阴影、相似背景和遮挡等 HSV 负样本验证。
- 未进行机械臂抓取、arm-base 外参标定或任何执行器控制。
