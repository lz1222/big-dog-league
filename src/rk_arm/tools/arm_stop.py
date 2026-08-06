#!/usr/bin/env python3
"""请求驱动停止；停止 JSON 未确认时会明确报告而非假称成功。"""
import rclpy
from rk_interfaces.srv import StopArm


def main():
    rclpy.init(); node = rclpy.create_node('arm_stop'); client = node.create_client(StopArm, 'arm/stop')
    if not client.wait_for_service(timeout_sec=2.0): raise RuntimeError('arm/stop unavailable')
    future = client.call_async(StopArm.Request()); rclpy.spin_until_future_complete(node, future, timeout_sec=3.0); print(future.result())
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
