#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source /home/unitree/rk_inspection_ws/install/setup.bash

pkill -f forwarder 2>/dev/null
sleep 0.5

# Start forwarder in background
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py &
FWD_PID=$!
sleep 2

# Send Twist via Python directly
python3 -c "
import rclpy, time
from geometry_msgs.msg import Twist
rclpy.init()
p = rclpy.create_node('t').create_publisher(Twist, '/navigation/cmd_vel', 10)
tw = Twist(); tw.linear.x = 0.25
print('forward...')
for i in range(30): p.publish(tw); time.sleep(0.1)
tw.linear.x = 0.0
for i in range(15): p.publish(tw); time.sleep(0.1)
print('turn...')
tw.angular.z = 0.50
for i in range(30): p.publish(tw); time.sleep(0.1)
print('done')
"

kill $FWD_PID 2>/dev/null
