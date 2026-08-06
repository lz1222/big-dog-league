#!/usr/bin/env python3
"""ROS 夹爪服务客户端。DEVELOPMENT DEFAULT 下请求会由驱动 fail closed。"""
import rclpy
from rk_interfaces.srv import SetArmGripper


def main():
    rclpy.init(); node = rclpy.create_node('arm_gripper_test'); client = node.create_client(SetArmGripper, 'arm/set_gripper')
    if not client.wait_for_service(timeout_sec=2.0): raise RuntimeError('arm/set_gripper unavailable')
    request = SetArmGripper.Request(); request.target = 0.0; request.max_speed = 0.0; request.timeout_sec = 0.0
    future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=3.0); print(future.result())
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
