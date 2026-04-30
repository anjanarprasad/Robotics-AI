#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        traj_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer = create_wall_timer(
            100ms,
            std::bind(&OffboardControl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Offboard control node started");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
    rclcpp::TimerBase::SharedPtr timer;

    int setpoint_counter_{0};

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub->publish(msg);
    }

    void timer_callback()
    {
        px4_msgs::msg::OffboardControlMode mode{};
        mode.position = true;
        mode.velocity = false;
        mode.acceleration = false;
        mode.attitude = false;
        mode.body_rate = false;
        mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_pub->publish(mode);

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position = {0.0f, 0.0f, -5.0f};
        sp.yaw = 0.0f;
        sp.timestamp = mode.timestamp;
        traj_pub->publish(sp);

        if (setpoint_counter_ == 10) {
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0f, 6.0f);

            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0f);

            RCLCPP_INFO(this->get_logger(), "Sent offboard mode and arm command");
        }

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