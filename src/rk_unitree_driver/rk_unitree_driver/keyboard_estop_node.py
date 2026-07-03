#!/usr/bin/env python3

import select
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from rk_unitree_driver.go2_motion_client import Go2MotionClient


class KeyboardEstopNode(Node):
    """Keyboard emergency stop for Go2 line navigation."""

    def __init__(self):
        super().__init__('keyboard_estop_node')

        self.declare_parameter('backend', Go2MotionClient.UNITREE_ROS2_BACKEND)
        self.declare_parameter('cmd_vel_topic', '/navigation/cmd_vel')
        self.declare_parameter('mission_stop_topic', '/mission/stop')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('estop_key', 'g')
        self.declare_parameter('zero_cmd_publish_count', 10)
        self.declare_parameter('zero_cmd_publish_period_sec', 0.05)
        self.declare_parameter('stop_publish_count', 5)
        self.declare_parameter('stop_publish_period_sec', 0.08)
        self.declare_parameter('stand_down_delay_sec', 0.6)
        self.declare_parameter('stand_down_repeat_count', 1)
        self.declare_parameter('stand_down_repeat_period_sec', 0.5)
        self.declare_parameter('send_damp_after_stand_down', False)
        self.declare_parameter('damp_delay_sec', 1.0)

        self.backend = self.string_parameter('backend')
        self.cmd_vel_topic = self.string_parameter('cmd_vel_topic')
        self.mission_stop_topic = self.string_parameter('mission_stop_topic')
        self.sport_request_topic = self.string_parameter('sport_request_topic')
        self.estop_key = self.string_parameter('estop_key')[:1].lower()
        self.zero_cmd_publish_count = self.positive_int_parameter(
            'zero_cmd_publish_count'
        )
        self.zero_cmd_publish_period_sec = self.nonnegative_float_parameter(
            'zero_cmd_publish_period_sec'
        )
        self.stop_publish_count = self.positive_int_parameter(
            'stop_publish_count'
        )
        self.stop_publish_period_sec = self.nonnegative_float_parameter(
            'stop_publish_period_sec'
        )
        self.stand_down_delay_sec = self.nonnegative_float_parameter(
            'stand_down_delay_sec'
        )
        self.stand_down_repeat_count = self.positive_int_parameter(
            'stand_down_repeat_count'
        )
        self.stand_down_repeat_period_sec = self.nonnegative_float_parameter(
            'stand_down_repeat_period_sec'
        )
        self.send_damp_after_stand_down = bool(
            self.get_parameter('send_damp_after_stand_down').value
        )
        self.damp_delay_sec = self.nonnegative_float_parameter(
            'damp_delay_sec'
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        self.mission_stop_publisher = self.create_publisher(
            Bool,
            self.mission_stop_topic,
            10
        )
        self.motion_client = Go2MotionClient(
            self,
            self.sport_request_topic,
            self.backend
        )

        self._estop_event = threading.Event()
        self._estop_started = False
        self._shutdown_event = threading.Event()
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop,
            name='keyboard_estop_reader',
            daemon=True
        )
        self._keyboard_thread.start()
        self._estop_timer = self.create_timer(0.05, self._run_estop_once)
        self._hold_stop_timer = self.create_timer(0.10, self._hold_stop)

        self.get_logger().warn(
            'Keyboard emergency stop armed: '
            f'press "{self.estop_key}" to stop and StandDown'
        )

    def _keyboard_loop(self):
        try:
            with open('/dev/tty', 'r') as tty_file:
                fd = tty_file.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    while not self._shutdown_event.is_set():
                        readable, _, _ = select.select([tty_file], [], [], 0.1)
                        if not readable:
                            continue
                        char = tty_file.read(1).lower()
                        if char == self.estop_key:
                            self._estop_event.set()
                            return
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except OSError as exc:
            self.get_logger().error(
                'Keyboard emergency stop cannot open /dev/tty: '
                f'{exc}. Run this launch from an interactive terminal.'
            )
        except termios.error as exc:
            self.get_logger().error(
                f'Keyboard emergency stop terminal setup failed: {exc}'
            )

    def _run_estop_once(self):
        if self._estop_started:
            return
        if not self._estop_event.is_set():
            return

        self._estop_started = True
        self.get_logger().error(
            f'Emergency key "{self.estop_key}" pressed: stopping Go2 now'
        )
        self.execute_estop_sequence()

    def execute_estop_sequence(self):
        self.publish_mission_stop()
        self.publish_repeated_zero_cmd()

        self.motion_client.send_repeated_stop(
            'keyboard emergency stop',
            self.stop_publish_count,
            self.stop_publish_period_sec
        )
        time.sleep(self.stand_down_delay_sec)

        for index in range(self.stand_down_repeat_count):
            self.motion_client.send_stand_down(
                f'keyboard emergency stop ({index + 1}/'
                f'{self.stand_down_repeat_count})'
            )
            if index + 1 < self.stand_down_repeat_count:
                time.sleep(self.stand_down_repeat_period_sec)

        if self.send_damp_after_stand_down:
            time.sleep(self.damp_delay_sec)
            self.motion_client.send_damp('keyboard emergency stop complete')

        self.get_logger().error('Emergency stop sequence finished')

    def _hold_stop(self):
        if not self._estop_started:
            return
        self.publish_mission_stop()
        self.cmd_vel_publisher.publish(Twist())

    def publish_mission_stop(self):
        msg = Bool()
        msg.data = True
        self.mission_stop_publisher.publish(msg)

    def publish_repeated_zero_cmd(self):
        zero = Twist()
        for _ in range(self.zero_cmd_publish_count):
            self.cmd_vel_publisher.publish(zero)
            time.sleep(self.zero_cmd_publish_period_sec)

    def destroy_node(self):
        self._shutdown_event.set()
        return super().destroy_node()

    def string_parameter(self, name):
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def nonnegative_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')
        return value

    def positive_int_parameter(self, name):
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f'{name} must be positive')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardEstopNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
