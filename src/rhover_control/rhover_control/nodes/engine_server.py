import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from rhover_control.rhover_control_interfaces import EngineState


class EngineServer(Node):
    def __init__(self):
        super().__init__('rhover_engine_manager')
        self.get_logger().info('Engine Manager startup')

        self.sub = self.create_subscription(String, 'rhover_engine', self.handler, 10)
        self.get_logger().info('Subscribed to /rhover_engine')

    def handler(self, msg):
        self.get_logger().info(f'Received data: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    node = EngineManagerNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
