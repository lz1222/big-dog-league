import rclpy
from rclpy.node import Node

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info("Line Follower Node Started")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()