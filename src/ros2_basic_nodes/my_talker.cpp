#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("my_talker"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::on_timer, this));
    RCLCPP_INFO(this->get_logger(), "my_talker started (publishing on /chatter)");
  }

private:
  void on_timer() {
    std_msgs::msg::String msg;
    msg.data = "hello " + std::to_string(count_++);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "pub: %s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}