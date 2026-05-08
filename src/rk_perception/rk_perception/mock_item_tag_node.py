#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rk_interfaces.msg import ItemTag, ItemTagArray


class MockItemTagNode(Node):
    """Publish fixed item tags for first-stage mock integration tests."""

    def __init__(self):
        super().__init__('mock_item_tag_node')
        self.publisher = self.create_publisher(
            ItemTagArray,
            '/perception/item_tags',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_mock_data)
        self.get_logger().info('Mock item tag node started')

    def publish_mock_data(self):
        msg = ItemTagArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        start_item = self._make_tag(1, 'start_item', 0.80, 0.10, 0.20)
        field_item = self._make_tag(2, 'field_item', 1.60, -0.20, 0.20)
        msg.tags.extend([start_item, field_item])

        self.publisher.publish(msg)
        self.get_logger().info('Publishing item tags: 2 tags')

    def _make_tag(self, tag_id, item_type, x, y, z):
        tag = ItemTag()
        tag.header.stamp = self.get_clock().now().to_msg()
        tag.header.frame_id = 'map'
        tag.tag_id = tag_id
        tag.item_type = item_type
        tag.pose.position.x = x
        tag.pose.position.y = y
        tag.pose.position.z = z
        tag.pose.orientation.w = 1.0
        tag.confidence = 0.90
        return tag


def main(args=None):
    rclpy.init(args=args)
    node = MockItemTagNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
