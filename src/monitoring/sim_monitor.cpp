#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"

static double clock_to_sec(const rosgraph_msgs::msg::Clock &c) {
  return static_cast<double>(c.clock.sec) + 1e-9 * static_cast<double>(c.clock.nanosec);
}

class SimMonitor : public rclcpp::Node {
public:
  SimMonitor() : Node("sim_monitor") {
    // Parameters (so you can point to any topics without recompiling)
    clock_topic_ = this->declare_parameter<std::string>("clock_topic", "/world/default/clock");
    imu_topic_   = this->declare_parameter<std::string>("imu_topic", "/imu");
    report_period_sec_ = this->declare_parameter<double>("report_period_sec", 1.0);

    // Clock is usually reliable, but best_effort is also safe in many sims.
    // Use KeepLast to avoid queue buildup.
    auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();

    // IMU is sensor-data: best-effort QoS is critical to avoid mismatch.
    auto imu_qos = rclcpp::SensorDataQoS();  // best_effort + small queue

    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      clock_topic_, clock_qos,
      [this](const rosgraph_msgs::msg::Clock &msg) { on_clock(msg); });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      [this](const sensor_msgs::msg::Imu &msg) { on_imu(msg); });

    report_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(report_period_sec_),
      [this]() { report(); });

    RCLCPP_INFO(this->get_logger(), "sim_monitor started");
    RCLCPP_INFO(this->get_logger(), "  clock_topic: %s", clock_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  imu_topic:   %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  report_period_sec: %.2f", report_period_sec_);
  }

private:
  void on_clock(const rosgraph_msgs::msg::Clock &msg) {
    const double t = clock_to_sec(msg);

    if (last_clock_t_ > 0.0) {
      const double dt = t - last_clock_t_;
      if (dt > 0.0) {
        clock_dt_sum_ += dt;
        clock_dt_n_++;
      }
    }
    last_clock_t_ = t;
    last_clock_rx_wall_ = this->now();  // wall time when we received last clock msg
  }

  void on_imu(const sensor_msgs::msg::Imu &m) {
    const auto &a = m.linear_acceleration;
    const auto &g = m.angular_velocity;

    accel_mag_ = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    gyro_mag_  = std::sqrt(g.x*g.x + g.y*g.y + g.z*g.z);

    imu_seen_ = true;
    last_imu_rx_wall_ = this->now();  // wall time when we received last IMU msg
  }

  void report() {
    // Compute clock rate from sim time deltas
    double hz = 0.0;
    if (clock_dt_n_ > 0) {
      const double mean_dt = clock_dt_sum_ / static_cast<double>(clock_dt_n_);
      if (mean_dt > 1e-9) hz = 1.0 / mean_dt;
    }

    // Basic “staleness” check (in wall time)
    const auto now_wall = this->now();
    const double imu_age = imu_seen_ ? (now_wall - last_imu_rx_wall_).seconds() : -1.0;

    if (imu_seen_ && imu_age >= 0.0 && imu_age < 5.0) {
      RCLCPP_INFO(this->get_logger(),
                  "clock_hz=%.1f | imu |accel|=%.3f m/s^2 | |gyro|=%.6f rad/s (imu_age=%.2fs)",
                  hz, accel_mag_, gyro_mag_, imu_age);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "clock_hz=%.1f | imu: not received yet (or stale) | imu_age=%.2fs",
                  hz, imu_age);
    }

    // reset accumulators each report interval
    clock_dt_sum_ = 0.0;
    clock_dt_n_ = 0;
  }

  // Params
  std::string clock_topic_;
  std::string imu_topic_;
  double report_period_sec_{1.0};

  // Subs/timer
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  // Clock stats
  double last_clock_t_{-1.0};
  double clock_dt_sum_{0.0};
  int clock_dt_n_{0};
  rclcpp::Time last_clock_rx_wall_{0, 0, RCL_ROS_TIME};

  // IMU stats
  bool imu_seen_{false};
  double accel_mag_{0.0};
  double gyro_mag_{0.0};
  rclcpp::Time last_imu_rx_wall_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimMonitor>());
  rclcpp::shutdown();
  return 0;
}