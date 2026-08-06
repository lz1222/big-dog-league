#!/bin/bash
# 完整迷宫自主导航: 直走→拐角检测→圆弧转弯→继续→五弯出口
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash
source /home/unitree/rk_inspection_ws/install/setup.bash

pkill -f forwarder 2>/dev/null; sleep 0.5

python3 /home/unitree/rk_inspection_ws/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/cmd_vel_udp_forwarder.py &
sleep 2

echo "=== 迷宫全自主 L-L-R-R-L ==="
timeout 180 python3 -c "
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

# --- Config ---
ROUTE = ['LEFT', 'LEFT', 'RIGHT', 'RIGHT', 'LEFT']
STOP_DIST = 0.60
TURN_YAWS = {'LEFT': +90.0, 'RIGHT': -90.0}
CORNER_HDG_THRESH = 30.0  # heading change > this = corner

# --- Init ---
rclpy.init()
pub = rclpy.create_node('walker').create_publisher(Twist, '/navigation/cmd_vel', 10)
chk = rclpy.create_node('sensor')
lock = threading.Lock()
cloud_data = []; odo_yaw = [0.0]; odo_yaw0 = [None]; imu_wz = [0.0]
grid = LocalOccupancyGrid(LocalGridConfig())
we = LidarWallExtractor(LocalGridConfig())
hc = HeadingController(HeadingControllerConfig())
ld = LidarDistanceConfig(min_cluster_points=3)
frame = [0]

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

# --- State machine ---
state = 'CRUISE'
turn_idx = 0
turn_start_yaw = 0.0
turn_yaw_target = 0.0
turn_dir = ''
reacquire_count = 0
last_log = 0.0

print(f'迷宫全自主启动。路线: {ROUTE}')
print(f'状态: {state}  当前弯: {turn_idx+1}/{len(ROUTE)}')

t0 = time.time()
while time.time() - t0 < 150:
    rclpy.spin_once(chk, timeout_sec=0.05)
    frame[0] += 1
    if frame[0] % 8 != 0: continue

    with lock:
        pts = list(cloud_data); o_yaw = odo_yaw[0]; o_yaw0 = odo_yaw0[0]; i_wz = imu_wz[0]
    if not pts: continue

    now = time.time()
    cloud = [Point3D(x=p[0],y=p[1],z=p[2]) for p in pts]
    filt = voxel_downsample(filter_point_cloud(cloud, ld), 0.02)
    fp = [p for p in filt if abs(math.degrees(math.atan2(p.y,p.x))) <= 30 and p.z > 0.005]
    if not fp: continue

    sd = compute_hard_distance(fp, SECTOR_FRONT, ld, now, now)
    if not sd.valid: continue

    front = sd.front_clearance
    for pt in filt: grid.mark_occupied(pt.x, pt.y)

    rel_yaw = hc._normalize_angle(o_yaw - o_yaw0) if o_yaw0 else 0.0
    out = we.extract(grid, now)
    model = we.build_corridor_model(out.wall_segments, now)
    heading_deg = math.degrees(model.corridor_heading) if model.corridor_heading else 0.0

    heading_state = hc.compute(
        corridor_heading=model.corridor_heading if model.valid else None,
        wall_confidence=model.confidence, wall_age_sec=0.05,
        odom_yaw=rel_yaw, odom_age_sec=0.05,
        imu_wz=i_wz, imu_age_sec=0.05,
        left_clearance=model.left_wall_distance if math.isfinite(model.left_wall_distance) else 1.0,
        right_clearance=model.right_wall_distance if math.isfinite(model.right_wall_distance) else 1.0,
        now_sec=now,
    )

    tw = Twist()
    log_flag = (time.time() - last_log > 1.5)

    # ====== CRUISE ======
    if state == 'CRUISE':
        # Corner detection
        corner_detected = False
        if front < STOP_DIST and front > 0.01:
            corner_detected = True
            print(f'拐角检测: 前墙={front:.2f}m < {STOP_DIST}m')

        if turn_idx >= len(ROUTE):
            state = 'DONE'
            print('全部弯完成! 停车.')
            continue

        if corner_detected:
            state = 'STOP_AND_SCAN'
            grid = LocalOccupancyGrid(LocalGridConfig())  # reset grid
            print(f'进入 STOP_AND_SCAN  弯{turn_idx+1}')
            continue

        # Forward cruise with heading correction
        tw.linear.x = 0.25
        tw.angular.z = heading_state.wz_reference if heading_state.valid else 0.0
        tw.angular.z = max(-0.30, min(0.30, tw.angular.z))

        if log_flag:
            print(f'CRUISE | front={front:.2f}m hdg={heading_deg:+.1f}deg wz={tw.angular.z:+.2f}')

    # ====== STOP_AND_SCAN ======
    elif state == 'STOP_AND_SCAN':
        # Use known route direction, verify with wall model if available
        turn_dir = ROUTE[turn_idx]
        turn_yaw_target = TURN_YAWS[turn_dir]
        turn_start_yaw = rel_yaw
        reacquire_count = 0
        state = 'ARC_TURN'
        hdg_info = f' hdg={heading_deg:.1f}deg' if model.valid else ''
        print(f'开始 {turn_dir} 弯  目标 {turn_yaw_target:.0f}deg  start_yaw={math.degrees(turn_start_yaw):.1f}deg{hdg_info}')

    # ====== ARC_TURN ======
    elif state == 'ARC_TURN':
        yaw_progress = math.degrees(hc._normalize_angle(rel_yaw - turn_start_yaw))
        remaining = abs(turn_yaw_target) - abs(yaw_progress)

        if remaining < 5.0:  # within 5 degrees of target
            state = 'REACQUIRE'
            grid = LocalOccupancyGrid(LocalGridConfig())
            print(f'{turn_dir} 弯完成! yaw={yaw_progress:.1f}deg')
            continue

        # Arc turn Twist — need forward speed for Go2 to turn
        wz_sign = 1.0 if turn_dir == 'LEFT' else -1.0
        wz = wz_sign * 0.40  # constant moderate turn rate
        tw.linear.x = 0.15  # need forward speed for arc turn
        tw.angular.z = wz

        if log_flag:
            print(f'{turn_dir}_TURN | yaw={yaw_progress:+.1f}deg rem={remaining:.0f}deg wz={wz:+.2f}')

    # ====== REACQUIRE ======
    elif state == 'REACQUIRE':
        if model.valid and model.corridor_heading is not None and abs(heading_deg) < 15.0:
            reacquire_count += 1
            if reacquire_count >= 3:
                turn_idx += 1
                state = 'CRUISE'
                print(f'新走廊确认! hdg={heading_deg:.1f}deg  弯{turn_idx}/{len(ROUTE)}完成  继续直走')
                continue
        else:
            reacquire_count = 0

        # Slow forward during reacquisition
        tw.linear.x = 0.10
        if log_flag: print(f'REACQUIRE | hdg={heading_deg:+.1f}deg count={reacquire_count}/3')

    # ====== DONE ======
    elif state == 'DONE':
        print('迷宫完成! 出口停车.')
        break

    pub.publish(tw)

# Final stop
tw = Twist()
for i in range(20): pub.publish(tw); time.sleep(0.1)

rclpy.shutdown()
print(f'自主行走结束。最终状态: {state}  完成弯数: {turn_idx}/{len(ROUTE)}')
"

pkill -f forwarder 2>/dev/null
echo "完成。"
