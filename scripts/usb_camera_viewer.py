#!/usr/bin/env python3

"""USB UVC 相机快速查看工具 — 连续抓帧 / 拍快照 / ROS bridge.

用法:
  # 持续抓帧存到 /tmp/frames/（30 秒后自动停止）
  python3 scripts/usb_camera_viewer.py stream --device 0 --duration 30

  # 拍几张快照
  python3 scripts/usb_camera_viewer.py snap --device 0 --path /tmp/snap.jpg

  # 以 ROS2 Image 发布画面（需要先 source ROS2 环境）
  python3 scripts/usb_camera_viewer.py ros --device 0 --topic /usb_camera/image_raw

  # 按 q 键退出（需要有显示器）
  python3 scripts/usb_camera_viewer.py show --device 0
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2


def open_camera(device: int = 0, width: int = 640, height: int = 480):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera device {device}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def cmd_snap(device: int, path: str):
    cap = open_camera(device)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("Failed to capture frame", file=sys.stderr)
        sys.exit(1)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path, frame)
    print(f"Saved {frame.shape[1]}x{frame.shape[0]} to {path}")


def cmd_stream(device: int, duration: float, output_dir: str):
    cap = open_camera(device)
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Camera {device}: {w}x{h} @ {fps:.1f}fps")
    print(f"Saving frames to {out_path}/  (Ctrl+C to stop)")

    start = time.time()
    count = 0
    try:
        while True:
            if duration > 0 and time.time() - start > duration:
                break
            ret, frame = cap.read()
            if ret:
                fname = out_path / f"frame_{count:06d}.jpg"
                cv2.imwrite(str(fname), frame)
                count += 1
                if count % 10 == 0:
                    elapsed = time.time() - start
                    print(f"  {count} frames, {elapsed:.1f}s")
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start
    cap.release()
    print(f"Done: {count} frames in {elapsed:.1f}s ({count/elapsed:.1f} fps)")


def cmd_show(device: int):
    cap = open_camera(device)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {device}: {w}x{h}  |  Press 'q' to quit, 's' to save snapshot")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.putText(frame, time.strftime("%H:%M:%S"), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("USB Camera (q=quit, s=snap)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            snap_path = f"/tmp/camera_snap_{time.strftime('%H%M%S')}.jpg"
            cv2.imwrite(snap_path, frame)
            print(f"Snap → {snap_path}")

    cap.release()
    cv2.destroyAllWindows()


def cmd_ros(device: int, topic: str):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("ROS2 not available. Source your ROS2 workspace first.", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node("usb_camera_viewer")
    pub = node.create_publisher(Image, topic, 10)
    bridge = CvBridge()
    cap = open_camera(device)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    node.get_logger().info(f"Publishing {w}x{h} to {topic}")

    def timer_cb():
        ret, frame = cap.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "usb_camera"
            pub.publish(msg)

    timer = node.create_timer(1/15.0, timer_cb)  # 15 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="USB UVC Camera Viewer")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("snap", help="Take a single snapshot")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--path", default="/tmp/camera_snap.jpg")

    p = sub.add_parser("stream", help="Continuously save frames")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--duration", type=float, default=30, help="Seconds (0=forever)")
    p.add_argument("--output-dir", default="/tmp/frames")

    p = sub.add_parser("show", help="Display live video (needs display)")
    p.add_argument("--device", type=int, default=0)

    p = sub.add_parser("ros", help="Publish to ROS2 Image topic")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--topic", default="/usb_camera/image_raw")

    args = parser.parse_args()

    if args.cmd == "snap":
        cmd_snap(args.device, args.path)
    elif args.cmd == "stream":
        cmd_stream(args.device, args.duration, args.output_dir)
    elif args.cmd == "show":
        cmd_show(args.device)
    elif args.cmd == "ros":
        cmd_ros(args.device, args.topic)


if __name__ == "__main__":
    main()
