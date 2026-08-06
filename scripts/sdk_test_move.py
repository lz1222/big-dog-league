#!/usr/bin/env python3
"""Quick SDK movement test — single script."""
import os, sys
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('LD_LIBRARY_PATH', '/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib')
import rclpy, time
from geometry_msgs.msg import Twist

rclpy.init()
pub = rclpy.create_node('sdk_test').create_publisher(Twist, '/navigation/cmd_vel', 10)
tw = Twist()

print('前进 vx=0.25 x 3秒...')
tw.linear.x = 0.30
for i in range(30): pub.publish(tw); time.sleep(0.1)

tw.linear.x = 0.0
for i in range(15): pub.publish(tw); time.sleep(0.1)

print('左转 wz=0.50 x 3秒...')
tw.angular.z = 0.50
for i in range(30): pub.publish(tw); time.sleep(0.1)

tw.angular.z = 0.0
for i in range(15): pub.publish(tw); time.sleep(0.1)

print('完成。狗动了吗?')
