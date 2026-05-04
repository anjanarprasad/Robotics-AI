import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

# This node reads simulated leak-risk values and converts them into mission states.
class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
         # Publisher for sending the selected mission state.
        # The mission state is published on the /mission_state topic.
        
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        
           # Subscriber for receiving simulated leak-risk values.
        # The node listens to the /leak_risk topic and calls risk_callback
        # whenever a new risk value is received.
        
        self.subscription = self.create_subscription(
            Float32,
            '/leak_risk',
            self.risk_callback,
            10
        )
         # Log message to show that the decision node has started.
        self.get_logger().info('Decision node started')

    def risk_callback(self, msg):
        # Read the leak-risk value from the incoming message
        risk = msg.data
        
       # Rule-based decision logic
        if risk < 0.4:
            state = "SEARCH"
        elif risk < 0.7:
            state = "INVESTIGATE"
        else:
            state = "CONFIRM_HOVER"
            
        # Create a String message and store the selected mission state.
        out = String()
        out.data = state
        # Publish the mission state so other nodes, such as the mission command node,
        # can use it to control the drone behavior.
        self.state_pub.publish(out)
         # Print the current risk value and selected mission state in the terminal.
        self.get_logger().info(f'Risk: {risk:.2f} -> Mission State: {state}')


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library.
    node = DecisionNode()   # Create and run the decision node.
    rclpy.spin(node)        # Keep the node running so it can continue receiving risk values.
    node.destroy_node()     #  shut down the node when the program stops.
    rclpy.shutdown()


if __name__ == '__main__':
    main()

