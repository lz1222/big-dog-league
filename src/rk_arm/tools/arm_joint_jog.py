#!/usr/bin/env python3
"""ROS 服务客户端；驱动拒绝所有默认运动请求，脚本绝不创建 DDS writer。"""
import argparse
import rclpy
from rk_interfaces.srv import JogArmJoint


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('joint_index', type=int); parser.add_argument('direction', type=float); args = parser.parse_args()
    rclpy.init(); node = rclpy.create_node('arm_joint_jog')
    client = node.create_client(JogArmJoint, 'arm/jog_joint')
    if not client.wait_for_service(timeout_sec=2.0): raise RuntimeError('arm/jog_joint unavailable')
    request = JogArmJoint.Request(); request.joint_index = args.joint_index; request.direction = args.direction; request.step = 0.0; request.max_speed = 0.0; request.timeout_sec = 0.0
    future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=3.0); print(future.result())
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
