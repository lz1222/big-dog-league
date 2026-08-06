#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source /home/unitree/rk_inspection_ws/install/setup.bash

pkill -f forwarder 2>/dev/null
sleep 0.5

# Start forwarder
python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py &
sleep 2

echo "=== 自主行走(感知+停车) ==="
timeout 20 python3 -c "
import rclpy, math, time, threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
import sys; sys.path.insert(0, '/home/unitree/rk_inspection_ws/src/rk_maze')
from rk_maze.lidar_distance_core import *
from rk_maze.lidar_wall_extractor import LidarWallExtractor
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig

rclpy.init()
pub = rclpy.create_node('walker').create_publisher(Twist, '/navigation/cmd_vel', 10)
chk = rclpy.create_node('sensor')
lock = threading.Lock()
cloud_data = []; odo_yaw = [0.0]; odo_yaw0 = [None]; imu_wz = [0.0]
grid = LocalOccupancyGrid(LocalGridConfig())
we = LidarWallExtractor(LocalGridConfig())
hc = HeadingController(HeadingControllerConfig())
ld = LidarDistanceConfig(min_cluster_points=3)
frame = [0]; last_log = [0]

def on_c(msg):
    pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
    with lock: cloud_data.clear(); cloud_data.extend([(float(p[0]),float(p[1]),float(p[2])) for p in pts if all(math.isfinite(float(v)) for v in p)])

def on_o(msg):
    q = msg.pose.pose.orientation; y = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    with lock:
        odo_yaw[0] = y
        if odo_yaw0[0] is None: odo_yaw0[0] = y

def on_i(msg):
    with lock: imu_wz[0] = msg.angular_velocity.z

chk.create_subscription(PointCloud2, '/utlidar/cloud_base', on_c, 10)
chk.create_subscription(Odometry, '/utlidar/robot_odom', on_o, 10)
chk.create_subscription(Imu, '/utlidar/imu', on_i, 10)

print('自主行走启动。前墙<0.5m自动停。')
stopped = False

t0 = time.time()
while time.time() - t0 < 18:
    rclpy.spin_once(chk, timeout_sec=0.05)
    frame[0] += 1
    if frame[0] % 8 != 0: continue

    with lock:
        pts = list(cloud_data); o_yaw = odo_yaw[0]; o_yaw0 = odo_yaw0[0]; i_wz = imu_wz[0]

    if not pts: continue

    cloud = [Point3D(x=p[0],y=p[1],z=p[2]) for p in pts]
    filt = voxel_downsample(filter_point_cloud(cloud, ld), 0.02)
    fp = [p for p in filt if abs(math.degrees(math.atan2(p.y,p.x))) <= 30 and p.z > 0.005]
    if not fp: continue

    sd = compute_hard_distance(fp, SECTOR_FRONT, ld, time.time(), time.time())
    if not sd.valid: continue

    front = sd.front_clearance
    for pt in filt: grid.mark_occupied(pt.x, pt.y)

    rel_yaw = hc._normalize_angle(o_yaw - o_yaw0) if o_yaw0 else 0.0

    out = we.extract(grid, time.time())
    model = we.build_corridor_model(out.wall_segments, time.time())
    heading = hc.compute(
        corridor_heading=model.corridor_heading if model.valid else None,
        wall_confidence=model.confidence, wall_age_sec=0.05,
        odom_yaw=rel_yaw, odom_age_sec=0.05,
        imu_wz=i_wz, imu_age_sec=0.05,
        left_clearance=model.left_wall_distance if math.isfinite(model.left_wall_distance) else 1.0,
        right_clearance=model.right_wall_distance if math.isfinite(model.right_wall_distance) else 1.0,
        now_sec=time.time(),
    )

    tw = Twist()

    if front < 0.50:
        if not stopped:
            print(f'STOP! 前墙={front:.2f}m < 0.50m')
            stopped = True
    else:
        stopped = False
        tw.linear.x = 0.20
        tw.angular.z = heading.wz_reference if heading.valid else 0.0
        tw.angular.z = max(-0.30, min(0.30, tw.angular.z))

    pub.publish(tw)

    if time.time() - last_log[0] > 2:
        last_log[0] = time.time()
        print(f'front={front:.2f}m wz={tw.angular.z:+.2f} hdg={math.degrees(model.corridor_heading) if model.corridor_heading else 0:+.1f}deg cells={len(grid.occupied_cells())} [{\"WALK\" if not stopped else \"STOP\"}]')

rclpy.shutdown()
print('自主行走结束。')
"

pkill -f forwarder 2>/dev/null
echo "完成。"
