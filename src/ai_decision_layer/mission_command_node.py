import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

# This node converts mission states into PX4 offboard movement commands.
class MissionCommandNode(Node):
    def __init__(self):
        super().__init__('mission_command_node')

        # Default mission state before any message is received.
        self.state = "SEARCH"
        # Counter is used to delay OFFBOARD mode and arming until PX4 receives setpoints
        self.counter = 0
        # Flags to make sure OFFBOARD mode and ARM commands are sent only once.
        self.offboard_started = False
        self.armed = False

         # Subscribe to the mission state published by the decision node.
        # The received state can be SEARCH, INVESTIGATE, or CONFIRM_HOVER.
        self.sub = self.create_subscription(
            String,
            '/mission_state',
            self.state_callback,
            10
        )
       # Publisher for sending offboard control mode messages to PX4.
        # This tells PX4 that position control will be used.
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        # Publisher for sending position setpoints to PX4.
        # The drone moves based on these target positions.
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        # Publisher for sending vehicle commands to PX4.
        # This is used for commands such as switching to OFFBOARD mode and arming.
        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )
       # Run the publish loop every 0.1 seconds.
        # This continuously sends offboard mode and setpoint messages to PX4.
        self.timer = self.create_timer(0.1, self.publish_loop)
        
         # Log message to show that the node has started.
        self.get_logger().info('Mission command node started')

    def state_callback(self, msg):
         # Update the current mission state whenever a new state is received.
        self.state = msg.data

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        # Create a PX4 vehicle command message.
        msg = VehicleCommand()
        # PX4 expects timestamp in microseconds.
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
         # Command parameters depend on the type of command being sent.
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command  # Set the PX4 command ID.
        # Target and source IDs for PX4 communication.
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True  # Mark this command as coming from an external ROS 2 node.
        self.command_pub.publish(msg) # Publish the command to PX4.

    def engage_offboard_mode(self):
         # Send command to switch PX4 into OFFBOARD mode.
        # param1 = 1 and param2 = 6 are used for PX4 custom OFFBOARD mode.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboard_started = True  # Mark OFFBOARD mode command as sent.
        self.get_logger().info('Sent OFFBOARD mode command') 

    def arm(self):
        # Send command to arm the drone.
        # param1 = 1.0 means ARM.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True # Mark ARM command as sent.
        self.get_logger().info('Sent ARM command')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode() # Create an offboard control mode message.
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # PX4 expects timestamp in microseconds.

        # Enable position control mode.
        # Other control modes are disabled in this project.
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        # Publish the offboard control mode message to PX4.
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        # Create a trajectory setpoint message.
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # PX4 expects timestamp in microseconds.

        # Select the drone position based on the current mission state.
        if self.state == "SEARCH":
            msg.position = [2.0, 0.0, -2.0] # SEARCH state: move to a search position.
            msg.yaw = 0.0

        elif self.state == "INVESTIGATE":
            msg.position = [1.0, 1.0, -2.0] # INVESTIGATE state: move toward a different point for inspection.
            msg.yaw = 0.8

        elif self.state == "CONFIRM_HOVER":
            msg.position = [0.0, 0.0, -2.0] # CONFIRM_HOVER state: hold position near the suspected high-risk area.
            msg.yaw = 0.0

        else:
            # Default safety fallback position if an unknown state is received.
            msg.position = [0.0, 0.0, -2.0]
            msg.yaw = 0.0
        # Publish the selected setpoint to PX4.
        self.setpoint_pub.publish(msg)

        # Print the current state and setpoint in the terminal.
        self.get_logger().info(f'State: {self.state} -> Setpoint: {msg.position}')

    def publish_loop(self):
        # Continuously publish offboard control mode and trajectory setpoints.
        # PX4 requires these messages to be sent repeatedly during offboard control.
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # PX4 requires setpoints before OFFBOARD mode
        if self.counter == 20 and not self.offboard_started:
            self.engage_offboard_mode()
            
       # PX4 requires setpoints to be sent before switching to OFFBOARD mode.
        # After about 20 cycles, send the OFFBOARD mode command.
        
        if self.counter == 30 and not self.armed:
            self.arm()  # After about 30 cycles, arm the drone.

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 
