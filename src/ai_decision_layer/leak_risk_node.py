import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LeakRiskNode(Node):
    def __init__(self):
        super().__init__('leak_risk_node')
        self.publisher_ = self.create_publisher(Float32, '/leak_risk', 10)
        self.timer = self.create_timer(0.5, self.publish_risk)
        self.risk = 0.0
        self.get_logger().info('Leak risk node started')

    def publish_risk(self):
        self.risk += 0.05
        if self.risk > 1.0:
            self.risk = 0.0

        msg = Float32()
        msg.data = float(self.risk)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published leak risk: {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = LeakRiskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
