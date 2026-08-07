#!/usr/bin/env python3

"""将ROS Twist命令通过本机UDP安全转发到隔离的Unitree SDK进程。"""

import math
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelUdpForwarder(Node):
    """将ROS速度命令隔离转发到Unitree SDK进程，并提供上游超时保护。"""

    DEFAULT_UDP_HOST = '127.0.0.1'
    DEFAULT_UDP_PORT = 15001
    DEFAULT_CMD_VEL_TOPIC = '/navigation/cmd_vel'
    DEFAULT_MAX_VX = 0.25
    DEFAULT_MAX_VY = 0.05
    DEFAULT_MAX_YAW = 0.60
    DEFAULT_DEADBAND = 0.01
    DEFAULT_TIMEOUT_SEC = 0.30

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
        # 50ms检查周期使0.30s超时的检测抖动不超过一个SDK输出周期。
        self.timer = self.create_timer(0.05, self.on_timer)

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
            self.get_logger().error(
                'non-finite cmd_vel rejected; sending stop'
            )
            self.send_stop_once(reason='nonfinite_command')
            return

        raw_values = (
            ('vx', msg.linear.x, self.max_vx),
            ('vy', msg.linear.y, self.max_vy),
            ('yaw', msg.angular.z, self.max_yaw),
        )
        for name, value, limit in raw_values:
            if abs(value) > limit:
                # 越界通常表示上游故障，静默截断会掩盖危险命令。
                self.get_logger().error(
                    f'{name}={value:.6f} exceeds limit={limit:.6f}; '
                    'sending stop'
                )
                self.send_stop_once(reason='out_of_range_command')
                return

        vx = self.apply_deadband(msg.linear.x)
        vy = self.apply_deadband(msg.linear.y)
        yaw = self.apply_deadband(msg.angular.z)

        if self.is_zero_cmd(vx, vy, yaw):
            self.send_stop_once(reason='zero_command')
            return

        self.send_cmd(vx, vy, yaw, reason='move_command')
        self.has_sent_stop = False

    def on_timer(self):
        if self.last_cmd_time is None:
            return

        if time.monotonic() - self.last_cmd_time >= self.timeout_sec:
            self.send_stop_once(reason='forwarder_watchdog')

    def send_stop(self):
        self.send_stop_once(force=True, reason='explicit_stop')

    def send_stop_once(self, force=False, reason='stop'):
        if self.has_sent_stop and not force:
            return

        self.send_cmd(0.0, 0.0, 0.0, reason=reason)
        self.has_sent_stop = True

    def send_cmd(self, vx, vy, yaw, reason):
        payload = f'{vx} {vy} {yaw}'.encode('utf-8')
        self.sock.sendto(payload, (self.udp_host, self.udp_port))
        print(
            f'[UDP] send vx={vx:.3f} vy={vy:.3f} yaw={yaw:.3f} '
            f'reason={reason}',
            flush=True
        )

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            return super().destroy_node()

    def apply_deadband(self, value):
        if abs(value) <= self.deadband:
            return 0.0
        return value

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
        node.send_stop_once(force=True, reason='forwarder_shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
