#!/usr/bin/env python3
"""Test: burst vx=0.35 for 1s then cruise 0.25"""
import rclpy, time
from geometry_msgs.msg import Twist
rclpy.init()
pub = rclpy.create_node('t').create_publisher(Twist,'/navigation/cmd_vel',10)
tw = Twist()
tw.linear.x = 0.40; print('BURST 0.40...')
for i in range(10): pub.publish(tw); time.sleep(0.1)
tw.linear.x = 0.25; print('CRUISE 0.25...')
for i in range(50): pub.publish(tw); time.sleep(0.1)
tw.linear.x = 0.0; print('STOP')
for i in range(20): pub.publish(tw); time.sleep(0.1)
print('狗移动了吗?')
