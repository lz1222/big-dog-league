#!/usr/bin/env python3

import math
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelUdpForwarder(Node):
    UDP_HOST = '127.0.0.1'
    UDP_PORT = 15001
    CMD_VEL_TOPIC = '/navigation/cmd_vel'
    MAX_VX = 0.20
    MAX_VY = 0.10
    MAX_YAW = 0.5
    DEADBAND = 0.01
    TIMEOUT_SEC = 1.0

    def __init__(self):
        super().__init__('cmd_vel_udp_forwarder')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_cmd_time = None
        self.has_sent_stop = False

        self.subscription = self.create_subscription(
            Twist,
            self.CMD_VEL_TOPIC,
            self.on_cmd_vel,
            10
        )
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            'cmd_vel_udp_forwarder started: '
            f'{self.CMD_VEL_TOPIC} -> '
            f'{self.UDP_HOST}:{self.UDP_PORT}'
        )

    def on_cmd_vel(self, msg):
        self.last_cmd_time = time.monotonic()

        if not self.is_finite_cmd(msg):
            self.send_stop_once()
            return

        vx = self.apply_deadband_and_limit(msg.linear.x, self.MAX_VX)
        vy = self.apply_deadband_and_limit(msg.linear.y, self.MAX_VY)
        yaw = self.apply_deadband_and_limit(msg.angular.z, self.MAX_YAW)

        if self.is_zero_cmd(vx, vy, yaw):
            self.send_stop_once()
            return

        self.send_cmd(vx, vy, yaw)
        self.has_sent_stop = False

    def on_timer(self):
        if self.last_cmd_time is None:
            return

        if time.monotonic() - self.last_cmd_time >= self.TIMEOUT_SEC:
            self.send_stop_once()

    def send_stop(self):
        self.send_stop_once(force=True)

    def send_stop_once(self, force=False):
        if self.has_sent_stop and not force:
            return

        self.send_cmd(0.0, 0.0, 0.0)
        self.has_sent_stop = True

    def send_cmd(self, vx, vy, yaw):
        payload = f'{vx} {vy} {yaw}'.encode('utf-8')
        self.sock.sendto(payload, (self.UDP_HOST, self.UDP_PORT))
        print(
            f'[UDP] send vx={vx:.3f} vy={vy:.3f} yaw={yaw:.3f}',
            flush=True
        )

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            return super().destroy_node()

    @classmethod
    def apply_deadband_and_limit(cls, value, limit):
        if abs(value) <= cls.DEADBAND:
            return 0.0
        return max(-limit, min(limit, value))

    @staticmethod
    def is_finite_cmd(msg):
        return (
            math.isfinite(msg.linear.x)
            and math.isfinite(msg.linear.y)
            and math.isfinite(msg.angular.z)
        )

    @staticmethod
    def is_zero_cmd(vx, vy, yaw):
        return vx == 0.0 and vy == 0.0 and yaw == 0.0


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelUdpForwarder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_once(force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
