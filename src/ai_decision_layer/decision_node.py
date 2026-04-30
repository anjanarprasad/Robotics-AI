import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        self.subscription = self.create_subscription(
            Float32,
            '/leak_risk',
            self.risk_callback,
            10
        )

        self.get_logger().info('Decision node started')

    def risk_callback(self, msg):
        risk = msg.data

        if risk < 0.4:
            state = "SEARCH"
        elif risk < 0.7:
            state = "INVESTIGATE"
        else:
            state = "CONFIRM_HOVER"

        out = String()
        out.data = state
        self.state_pub.publish(out)

        self.get_logger().info(f'Risk: {risk:.2f} -> Mission State: {state}')


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

