#!/usr/bin/env python3
"""MAZE_RUNTIME_STABILITY_V1 — LiDAR DDS→ROS2 桥接。

订阅 Unitree DDS rt/utlidar/cloud (PointCloud2)，
转换为 ROS2 sensor_msgs/PointCloud2 发布到 /utlidar/cloud。
"""

import sys, time, threading
sys.path.insert(0, '/home/unitree/Documents/NoMachine/unitree_sdk2_python')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


class LidarBridgeNode(Node):
    """将 Unitree DDS PointCloud2 桥接到 ROS2。"""

    def __init__(self, network_interface: str = "eth1"):
        super().__init__('lidar_dds_bridge')

        # ROS2 发布者
        self.pub = self.create_publisher(PointCloud2, '/utlidar/cloud', 10)

        # Unitree DDS 订阅
        ChannelFactoryInitialize(0, network_interface)
        self.sub = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self.sub.Init(self._on_dds_cloud, 10)

        self._seq = 0
        self._last_msg_time = time.monotonic()
        self.get_logger().info(f'LiDAR bridge started: rt/utlidar/cloud → /utlidar/cloud (iface={network_interface})')

    def _on_dds_cloud(self, dds_msg: PointCloud2_):
        """DDS 回调：转换并发布到 ROS2。"""
        now = time.monotonic()
        self._last_msg_time = now
        self._seq += 1

        # 构建 ROS2 PointCloud2
        ros_msg = PointCloud2()
        ros_msg.header = Header(
            stamp=RosTime(sec=int(now), nanosec=int((now % 1) * 1e9)),
            frame_id="base_link",
        )
        ros_msg.height = dds_msg.height
        ros_msg.width = dds_msg.width
        ros_msg.is_bigendian = dds_msg.is_bigendian
        ros_msg.point_step = dds_msg.point_step
        ros_msg.row_step = dds_msg.row_step
        ros_msg.is_dense = dds_msg.is_dense

        # 转换 fields
        for f in dds_msg.fields:
            pf = PointField()
            pf.name = str(f.name)
            pf.offset = f.offset
            pf.datatype = f.datatype
            pf.count = f.count
            ros_msg.fields.append(pf)

        # 复制数据
        ros_msg.data = bytes(dds_msg.data)

        self.pub.publish(ros_msg)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', default='eth1')
    args = parser.parse_args()

    rclpy.init()
    node = LidarBridgeNode(args.interface)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("LiDAR DDS→ROS2 bridge running. Ctrl-C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
