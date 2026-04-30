import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class MissionCommandNode(Node):
    def __init__(self):
        super().__init__('mission_command_node')

        self.state = "SEARCH"
        self.counter = 0
        self.offboard_started = False
        self.armed = False

        self.sub = self.create_subscription(
            String,
            '/mission_state',
            self.state_callback,
            10
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_loop)

        self.get_logger().info('Mission command node started')

    def state_callback(self, msg):
        self.state = msg.data

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def engage_offboard_mode(self):
        # PX4 custom mode command: param1=1, param2=6 means OFFBOARD
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboard_started = True
        self.get_logger().info('Sent OFFBOARD mode command')

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True
        self.get_logger().info('Sent ARM command')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.state == "SEARCH":
            msg.position = [2.0, 0.0, -2.0]
            msg.yaw = 0.0

        elif self.state == "INVESTIGATE":
            msg.position = [1.0, 1.0, -2.0]
            msg.yaw = 0.8

        elif self.state == "CONFIRM_HOVER":
            msg.position = [0.0, 0.0, -2.0]
            msg.yaw = 0.0

        else:
            msg.position = [0.0, 0.0, -2.0]
            msg.yaw = 0.0

        self.setpoint_pub.publish(msg)
        self.get_logger().info(f'State: {self.state} -> Setpoint: {msg.position}')

    def publish_loop(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # PX4 requires setpoints before OFFBOARD mode
        if self.counter == 20 and not self.offboard_started:
            self.engage_offboard_mode()

        if self.counter == 30 and not self.armed:
            self.arm()

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 
