#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

// This node sends basic PX4 Offboard control commands.
// It publishes offboard control mode, trajectory setpoints,
// and vehicle commands for OFFBOARD mode and arming.

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // This node sends basic PX4 Offboard control commands.
// It publishes offboard control mode, trajectory setpoints,
// and vehicle commands for OFFBOARD mode and arming.
        
        offboard_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        // Publisher for sending target position setpoints to PX4.
        // PX4 uses these setpoints to move the drone in simulation.
        traj_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        // Publisher for sending vehicle-level commands to PX4.
        // This is used for actions such as switching to OFFBOARD mode and arming.

        vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Timer runs every 100 milliseconds.
        // This keeps publishing setpoints continuously, which PX4 requires for Offboard mode.
        timer = create_wall_timer(
            100ms,
            std::bind(&OffboardControl::timer_callback, this));

        // Startup message for terminal verification.
        RCLCPP_INFO(this->get_logger(), "Offboard control node started");
    }

private:

// ROS 2 publishers used to communicate with PX4.
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;

// Timer used to run the control loop repeatedly.
    rclcpp::TimerBase::SharedPtr timer;

// Counter used to delay OFFBOARD mode and arming until PX4 receives setpoints first.
    int setpoint_counter_{0};

// Helper function to publish PX4 vehicle commands.
    // It is used for commands such as enabling OFFBOARD mode and arming the drone.
    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};

         // Command parameters depend on the type of PX4 command being sent.
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;  // PX4 command ID.
        msg.target_system = 1;    // Target and source system/component IDs for PX4 communication.
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true; // Mark the command as coming from an external ROS 2 node.
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;  // PX4 expects timestamps in microseconds.
        vehicle_command_pub->publish(msg);  // Publish the command to PX4.

    void timer_callback()
    {
        // Create and publish the OffboardControlMode message.
        // This tells PX4 to use position-based offboard control.
        px4_msgs::msg::OffboardControlMode mode{};
        mode.position = true;
        mode.velocity = false;
        mode.acceleration = false;
        mode.attitude = false;
        mode.body_rate = false;
        mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_pub->publish(mode);

        // Create and publish a trajectory setpoint.
        // The target position keeps the drone at x=0, y=0, z=-5 in NED coordinates.
        // In PX4 NED frame, negative z means upward altitude.

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position = {0.0f, 0.0f, -5.0f};
        sp.yaw = 0.0f;
        sp.timestamp = mode.timestamp;
        traj_pub->publish(sp);


        // After a few setpoints have already been sent,
        // switch PX4 into OFFBOARD mode and arm the drone.
        if (setpoint_counter_ == 10) {
            
            // Send command to switch PX4 into OFFBOARD mode.
            // param1 = 1 and param2 = 6 are used for PX4 custom OFFBOARD mode.
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0f, 6.0f);

            // Send command to arm the drone.
            // param1 = 1.0 means arm.
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0f);

            // Terminal message for verification.
            RCLCPP_INFO(this->get_logger(), "Sent offboard mode and arm command");
        }

        // Increase the counter until the OFFBOARD/ARM command has been sent.
        if (setpoint_counter_ < 11) {
            setpoint_counter_++;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
