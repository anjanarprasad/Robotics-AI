import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# This node publishes simulated leak-risk values for testing the decision pipeline.
class LeakRiskNode(Node):
    def __init__(self):
        super().__init__('leak_risk_node')

        # Create a publisher that sends Float32 risk values on the /leak_risk topic
        self.publisher_ = self.create_publisher(Float32, '/leak_risk', 10)

        # Create a timer that calls publish_risk() every 0.5 seconds.
        # This makes the node publish risk values continuously during simulation.
        self.timer = self.create_timer(0.5, self.publish_risk)

         # Initial simulated leak-risk value.
        self.risk = 0.0
        # Log message to show that the node has started.
        self.get_logger().info('Leak risk node started')

    def publish_risk(self):
        # Increase the simulated risk value step by step.
        self.risk += 0.05

        # Reset the risk value after it reaches 1.0.
        # This allows the test values to repeat continuously.
        if self.risk > 1.0:
            self.risk = 0.0
            
        # Create a Float32 message and store the current risk value.
        msg = Float32()
        msg.data = float(self.risk)
        
         # Publish the simulated risk value to leak risk.
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published leak risk: {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 Python client library.
    node = LeakRiskNode() # Create and run the leak risk publisher node.
    rclpy.spin(node)    # Keep the node running so it continues publishing risk values.
    node.destroy_node()  # Cleanly shut down the node when the program stops.
    rclpy.shutdown()


if __name__ == '__main__':
    main()
