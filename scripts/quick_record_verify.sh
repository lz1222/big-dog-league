#!/bin/bash
# 快速录包+验证一个位置
# 用法: bash quick_record_verify.sh <标签> <秒数>
set -euo pipefail
LABEL="${1:-pos}"
DURATION="${2:-20}"
BAG_DIR=~/rk_inspection_ws/evidence/fusion_bags/${LABEL}_$(date +%Y%m%d_%H%M%S)

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/cyclonedds/lib:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib
source /opt/ros/foxy/setup.bash

echo "RECORD: $LABEL (${DURATION}s) -> $BAG_DIR"
timeout ${DURATION} ros2 bag record -o "$BAG_DIR" /utlidar/cloud_base /utlidar/robot_odom /utlidar/imu 2>/dev/null || true

DB=$(ls "$BAG_DIR"/*.db3 2>/dev/null | head -1)
if [ -z "$DB" ]; then
    echo "ERROR: no db3 found"
    exit 1
fi

echo "VERIFY: $DB"
python3 -c "
import json, math, statistics, sqlite3, sys
sys.path.insert(0, 'src/rk_maze')
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rk_maze.lidar_distance_core import *
from rk_maze.local_occupancy_grid import LocalGridConfig, LocalOccupancyGrid
from rk_maze.lidar_wall_extractor import LidarWallExtractor
from rk_maze.heading_controller import HeadingController, HeadingControllerConfig

db = '$DB'
conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
conn.row_factory = sqlite3.Row
topic_map = {r['id']: (r['name'], r['type']) for r in conn.execute('SELECT id, name, type FROM topics')}
ld = LidarDistanceConfig()
gc = LocalGridConfig()
ext = LidarWallExtractor(gc)
hc = HeadingController(HeadingControllerConfig())
grid = LocalOccupancyGrid(gc)
has_imu = any('/imu' in t[0] for t in topic_map.values())
imu_wz, odo_yaw, odo_yaw0 = 0.0, 0.0, None
hard_dists, n = [], 0
for row in conn.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY id'):
    tname = topic_map.get(row['topic_id'], ('',''))[0]
    stamp = row['timestamp'] * 1e-9
    data = row['data']
    if tname == '/utlidar/cloud_base':
        n += 1
        if n > 300: break
        msg = deserialize_message(data, PointCloud2)
        pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        cloud = [Point3D(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts if all(math.isfinite(float(v)) for v in p)]
        filt = voxel_downsample(filter_point_cloud(cloud, ld), 0.02)
        hard = compute_all_hard_distances(filt, ld, stamp, stamp+0.01)
        f = hard.get(SECTOR_FRONT)
        if f and f.valid: hard_dists.append(f.hard_distance)
        for pt in filt: grid.mark_occupied(pt.x, pt.y)
    elif '/imu' in tname:
        msg = deserialize_message(data, Imu)
        imu_wz = msg.angular_velocity.z
    elif tname == '/utlidar/robot_odom':
        msg = deserialize_message(data, Odometry)
        q = msg.pose.pose.orientation
        odo_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if odo_yaw0 is None: odo_yaw0 = odo_yaw
conn.close()

out = ext.extract(grid, 0.0)
model = ext.build_corridor_model(out.wall_segments, 0.0)
state = hc.compute(
    corridor_heading=model.corridor_heading if model.valid else None,
    wall_confidence=model.confidence, wall_age_sec=0.01,
    odom_yaw=0.0, odom_age_sec=0.01,
    imu_wz=imu_wz, imu_age_sec=0.01 if has_imu else 999.0,
    left_clearance=model.left_wall_distance if math.isfinite(model.left_wall_distance) else 1.0,
    right_clearance=model.right_wall_distance if math.isfinite(model.right_wall_distance) else 1.0,
    now_sec=stamp,
)

dists = hard_dists
m = sorted(dists)[len(dists)//2] if dists else 0
s = statistics.stdev(dists) if len(dists) > 1 else 0
print(f'Bag={db.split(\"/\")[-2]}')
print(f'frames={n} front_med={m:.3f}m front_std={s:.3f}m imu_wz={imu_wz:.4f} imu={\"YES\" if has_imu else \"NO\"}')
print(f'grid_cells={len(grid.occupied_cells())} wall_segs={len(out.wall_segments)}')
print(f'corridor_valid={model.valid} heading={math.degrees(model.corridor_heading) if model.corridor_heading else 0:.1f}deg')
print(f'right_wall={model.right_wall_distance:.3f}m left_wall={model.left_wall_distance:.3f}m')
print(f'wz_ref={state.wz_reference:+.4f} mode={state.reason}')
ok = len(dists) > 5 and s < 0.20 and model.valid
print(f'STATUS: {\"OK\" if ok else \"CHECK\"}')
"
