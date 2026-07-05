#!/usr/bin/env python3

import math
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelUdpForwarder(Node):
    DEFAULT_UDP_HOST = '127.0.0.1'
    DEFAULT_UDP_PORT = 15001
    DEFAULT_CMD_VEL_TOPIC = '/navigation/cmd_vel'
    DEFAULT_MAX_VX = 0.60
    DEFAULT_MAX_VY = 0.10
    DEFAULT_MAX_YAW = 1.00
    DEFAULT_DEADBAND = 0.01
    DEFAULT_TIMEOUT_SEC = 1.0

    def __init__(self):
        super().__init__('cmd_vel_udp_forwarder')

        self.udp_host = self.declare_parameter(
            'udp_host',
            self.DEFAULT_UDP_HOST
        ).value
        self.udp_port = int(self.declare_parameter(
            'udp_port',
            self.DEFAULT_UDP_PORT
        ).value)
        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            self.DEFAULT_CMD_VEL_TOPIC
        ).value
        self.max_vx = float(self.declare_parameter(
            'max_vx',
            self.DEFAULT_MAX_VX
        ).value)
        self.max_vy = float(self.declare_parameter(
            'max_vy',
            self.DEFAULT_MAX_VY
        ).value)
        self.max_yaw = float(self.declare_parameter(
            'max_yaw',
            self.DEFAULT_MAX_YAW
        ).value)
        self.deadband = float(self.declare_parameter(
            'deadband',
            self.DEFAULT_DEADBAND
        ).value)
        self.timeout_sec = float(self.declare_parameter(
            'timeout_sec',
            self.DEFAULT_TIMEOUT_SEC
        ).value)

        self.validate_parameters()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_cmd_time = None
        self.has_sent_stop = False

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.on_cmd_vel,
            10
        )
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            'cmd_vel_udp_forwarder started: '
            f'{self.cmd_vel_topic} -> '
            f'{self.udp_host}:{self.udp_port}, '
            f'max_vx={self.max_vx:.3f}, '
            f'max_vy={self.max_vy:.3f}, '
            f'max_yaw={self.max_yaw:.3f}, '
            f'deadband={self.deadband:.3f}, '
            f'timeout={self.timeout_sec:.3f}'
        )

    def validate_parameters(self):
        if not self.udp_host:
            raise ValueError('udp_host must not be empty')
        if self.udp_port <= 0 or self.udp_port > 65535:
            raise ValueError('udp_port must be in range 1..65535')
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')

        positive_limits = {
            'max_vx': self.max_vx,
            'max_vy': self.max_vy,
            'max_yaw': self.max_yaw,
            'timeout_sec': self.timeout_sec,
        }
        for name, value in positive_limits.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a positive finite number')

        if not math.isfinite(self.deadband) or self.deadband < 0.0:
            raise ValueError('deadband must be a nonnegative finite number')

    def on_cmd_vel(self, msg):
        self.last_cmd_time = time.monotonic()

        if not self.is_finite_cmd(msg):
            self.send_stop_once()
            return

        vx = self.apply_deadband_and_limit(msg.linear.x, self.max_vx)
        vy = self.apply_deadband_and_limit(msg.linear.y, self.max_vy)
        yaw = self.apply_deadband_and_limit(msg.angular.z, self.max_yaw)

        if self.is_zero_cmd(vx, vy, yaw):
            self.send_stop_once()
            return

        self.send_cmd(vx, vy, yaw)
        self.has_sent_stop = False

    def on_timer(self):
        if self.last_cmd_time is None:
            return

        if time.monotonic() - self.last_cmd_time >= self.timeout_sec:
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
        self.sock.sendto(payload, (self.udp_host, self.udp_port))
        print(
            f'[UDP] send vx={vx:.3f} vy={vy:.3f} yaw={yaw:.3f}',
            flush=True
        )

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            return super().destroy_node()

    def apply_deadband_and_limit(self, value, limit):
        if abs(value) <= self.deadband:
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
