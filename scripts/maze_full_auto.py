#!/usr/bin/env python3
"""迷宫全自主: 直走→拐角检测→圆弧转弯→五弯→出口"""
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

ROUTE = ['LEFT', 'LEFT', 'RIGHT', 'RIGHT', 'LEFT']
TURN_YAWS = {'LEFT': +90.0, 'RIGHT': -90.0}
STOP_DIST = 0.20       # 前墙<20cm才触发
SIDE_OPEN = 0.50       # 侧向>50cm认为有开口
VX_CRUISE = 0.30
VX_TURN = 0.10
WZ_TURN = 0.50

rclpy.init()
pub = rclpy.create_node('maze').create_publisher(Twist, '/navigation/cmd_vel', 10)
chk = rclpy.create_node('s')
lock = threading.Lock()
cloud_data = []; odo_yaw = [0.0]; odo_yaw0 = [None]; imu_wz = [0.0]
grid = LocalOccupancyGrid(LocalGridConfig())
we = LidarWallExtractor(LocalGridConfig())
hc = HeadingController(HeadingControllerConfig())
ld = LidarDistanceConfig(min_cluster_points=3)
frame = [0]; last_log = [0.0]

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

state = 'CRUISE'
turn_idx = 0
turn_start_yaw = 0.0     # rel_yaw at turn start
turn_imu_yaw = 0.0       # IMU-integrated yaw during turn
turn_yaw_target = 0.0
turn_dir = ''
reacquire_count = 0
state_enter_frame = 0
last_imu_time = 0.0

print(f'迷宫全自主 L-L-R-R-L 启动')
print(f'状态: {state}  弯: {turn_idx+1}/{len(ROUTE)}')

t0 = time.time()
while time.time() - t0 < 120:
    rclpy.spin_once(chk, timeout_sec=0.02)
    frame[0] += 1
    if frame[0] % 6 != 0: continue

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

    tw = Twist()
    log_flag = (time.time() - last_log[0] > 1.5)

    # ====== CRUISE ======
    if state == 'CRUISE':
        if turn_idx >= len(ROUTE):
            state = 'DONE'; print('全部弯完成!'); break

        # Side clearance from dedicated LiDAR sectors (30-60deg each side)
        left_pts = [p.y for p in filt if 30 < math.degrees(math.atan2(p.y, p.x)) < 60]
        right_pts = [-p.y for p in filt if -60 < math.degrees(math.atan2(p.y, p.x)) < -30]
        left_cl = sorted(left_pts)[len(left_pts)//2] if left_pts else 0.0
        right_cl = sorted(right_pts)[len(right_pts)//2] if right_pts else 0.0
        # Front corner wall: close front AND left opens up (for LEFT turn)
        turn_left_ready = (0.01 < front < 0.40) and (left_cl > 0.50)
        turn_right_ready = (0.01 < front < 0.40) and (right_cl > 0.50)
        corner = (0.01 < front < 0.15) or turn_left_ready or turn_right_ready

        if corner:
            state = 'TURN'; state_enter_frame = frame[0]
            turn_dir = ROUTE[turn_idx]; turn_yaw_target = TURN_YAWS[turn_dir]
            turn_start_yaw = rel_yaw; turn_imu_yaw = 0.0; last_imu_time = now
            reacquire_count = 0
            grid = LocalOccupancyGrid(LocalGridConfig())
            trigger = 'wall' if front < STOP_DIST else 'side_open'
            print(f'拐角[{trigger}]! 前={front:.2f}m L={left_cl:.2f}m R={right_cl:.2f}m  弯{turn_idx+1}: {turn_dir}')
            continue

        tw.linear.x = VX_CRUISE
        tw.angular.z = min(0.30, max(-0.30, heading_deg * 0.02))

        if log_flag: print(f'CRUISE front={front:.2f}m L={left_cl:.2f}m R={right_cl:.2f}m hdg={heading_deg:+.1f}deg')

    # ====== TURN ======
    elif state == 'TURN':
        # Integrate IMU angular velocity for yaw tracking
        dt_imu = now - last_imu_time if last_imu_time > 0 else 0.05
        turn_imu_yaw += math.degrees(i_wz * dt_imu)
        last_imu_time = now

        # Use IMU yaw first, fall back to odom rel_yaw
        yaw_progress = abs(turn_imu_yaw) if abs(turn_imu_yaw) > 2.0 else abs(math.degrees(hc._normalize_angle(rel_yaw - turn_start_yaw)))
        remaining = abs(turn_yaw_target) - yaw_progress

        if remaining < 15.0 and frame[0] - state_enter_frame > 20:
            turn_idx += 1; state = 'CRUISE'; grid = LocalOccupancyGrid(LocalGridConfig())
            print(f'弯{turn_idx}完成! imu_yaw={turn_imu_yaw:.1f}deg odom_yaw={math.degrees(hc._normalize_angle(rel_yaw - turn_start_yaw)):.1f}deg')
            continue

        wz_sign = 1.0 if turn_dir == 'LEFT' else -1.0
        tw.linear.x = VX_TURN
        tw.angular.z = wz_sign * WZ_TURN

        if log_flag: print(f'{turn_dir}_TURN imu={turn_imu_yaw:+.1f}deg odo={math.degrees(hc._normalize_angle(rel_yaw - turn_start_yaw)):+.1f}deg rem={remaining:.0f}deg')

    # ====== DONE ======
    elif state == 'DONE':
        print('迷宫完成!'); break

    for _ in range(3): pub.publish(tw)  # publish 3x for reliability

# Final stop
tw = Twist()
for i in range(30): pub.publish(tw); time.sleep(0.05)
print(f'结束。状态={state} 完成弯数={turn_idx}/{len(ROUTE)}')
