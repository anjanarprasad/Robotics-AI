#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This node subscribes to the chatter topic and prints received messages.

class Listener : public rclcpp::Node {
public:
  Listener() : Node("my_listener") {

    // Create a subscriber for the "chatter" topic.
    // The node listens for String messages published on this topic.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](const std_msgs::msg::String &msg) {

        // This callback runs whenever a new message is received.
        // It prints the received message data in the terminal.
        RCLCPP_INFO(this->get_logger(), "sub: %s", msg.data.c_str());
      });

    // Print a startup message to confirm the listener node is running.
    RCLCPP_INFO(this->get_logger(), "my_listener started (subscribing to /chatter)");
  }

private:
// Subscriber object for receiving messages from the "chatter" topic.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
