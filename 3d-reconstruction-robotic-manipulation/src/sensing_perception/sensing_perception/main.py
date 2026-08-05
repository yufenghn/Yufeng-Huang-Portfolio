import rclpy
from rclpy.node import Node

class CoreNode(Node):
    def __init__(self):
        super().__init__('core_node')
        self.get_logger().info('Sensing & Perception node started.')

def main(args=None):
    rclpy.init(args=args)
    node = CoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

