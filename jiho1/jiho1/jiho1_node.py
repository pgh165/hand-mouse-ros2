import rclpy
from rclpy.node import Node


class Jiho1Node(Node):
    def __init__(self):
        super().__init__('jiho1_node')
        self.get_logger().info('jiho1_node started')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello from jiho1!')


def main(args=None):
    rclpy.init(args=args)
    node = Jiho1Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
