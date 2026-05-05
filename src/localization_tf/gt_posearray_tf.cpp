#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;

// This node reads pose data from a Gazebo PoseArray topic.
// It converts the selected pose into:
// 1. a TF transform between odom and base_link
// 2. an odometry message on /odom
class GtPoseArrayTfNode : public rclcpp::Node
{
public:
  GtPoseArrayTfNode()
  : Node("gt_posearray_tf")
  {
    // DO NOT declare use_sim_time here.
    // It already exists on every ROS 2 node.

    this->declare_parameter<std::string>("input_topic", "/gt_pose_array");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("parent_frame", "odom");
    this->declare_parameter<std::string>("child_frame", "base_link");
    this->declare_parameter<int>("pose_index", 0);
    this->declare_parameter<bool>("publish_odom", true);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("cov_pose", 0.01);
    this->declare_parameter<double>("cov_twist", 0.05);
    this->declare_parameter<bool>("zero_twist", true);


     // Read parameter values after declaring them.
    input_topic_  = this->get_parameter("input_topic").as_string();
    odom_topic_   = this->get_parameter("odom_topic").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_  = this->get_parameter("child_frame").as_string();
    pose_index_   = this->get_parameter("pose_index").as_int();
    publish_odom_ = this->get_parameter("publish_odom").as_bool();
    publish_tf_   = this->get_parameter("publish_tf").as_bool();
    cov_pose_     = this->get_parameter("cov_pose").as_double();
    cov_twist_    = this->get_parameter("cov_twist").as_double();
    zero_twist_   = this->get_parameter("zero_twist").as_bool();

    // Create a TF broadcaster.
    // This is used to publish the transform between parent_frame and child_frame.
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create odometry publisher only if odometry publishing is enabled.
    if (publish_odom_) {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10));
    }

    // Subscribe to the Gazebo pose array topic.
    // SensorDataQoS is used because pose data from simulation behaves like sensor/runtime data.
    sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GtPoseArrayTfNode::poseArrayCallback, this, _1));

    // Create a watchdog timer.
    // This checks whether pose messages are still being received.
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&GtPoseArrayTfNode::watchdogCallback, this));

    // Store the current time as the initial last-received message time.
    last_msg_time_wall_ = this->now();

    // Print startup information in the terminal.
    RCLCPP_INFO(
      this->get_logger(),
      "gt_posearray_tf started | input_topic=%s | odom_topic=%s | %s->%s | pose_index=%d | publish_tf=%s | publish_odom=%s",
      input_topic_.c_str(),
      odom_topic_.c_str(),
      parent_frame_.c_str(),
      child_frame_.c_str(),
      pose_index_,
      publish_tf_ ? "true" : "false",
      publish_odom_ ? "true" : "false");
  }

private:
  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Update the last message time whenever a PoseArray is received.
    last_msg_time_wall_ = this->now();

    // Check whether the incoming message pointer is valid.
    if (!msg) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received null PoseArray pointer.");
      return;
    }

    // Check whether the PoseArray contains at least one pose.
    if (msg->poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received PoseArray with zero poses.");
      return;
    }

    // Check whether the selected pose index is valid.
    // This prevents reading outside the PoseArray.
    if (pose_index_ < 0 || static_cast<size_t>(pose_index_) >= msg->poses.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "pose_index=%d out of range for PoseArray size=%zu",
        pose_index_, msg->poses.size());
      return;
    }

    // Select the drone pose from the PoseArray.
    const auto & pose = msg->poses[pose_index_];

    // Use the timestamp from the incoming message.
    // If the message timestamp is empty, use the current node time.
    builtin_interfaces::msg::Time stamp = msg->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = this->get_clock()->now();
    }

    // Publish TF transform if enabled.
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;

      // Fill the transform header.
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = parent_frame_;
      tf_msg.child_frame_id = child_frame_;

      // Copy position from the selected Gazebo pose.
      tf_msg.transform.translation.x = pose.position.x;
      tf_msg.transform.translation.y = pose.position.y;
      tf_msg.transform.translation.z = pose.position.z;

       // Copy orientation from the selected Gazebo pose.
      tf_msg.transform.rotation = pose.orientation;
      // Broadcast the transform to the ROS 2 TF tree.
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // Publish odometry message if enabled.
    if (publish_odom_) {
      nav_msgs::msg::Odometry odom_msg;

      // Fill odometry header and frame information.
      odom_msg.header.stamp = stamp;
      odom_msg.header.frame_id = parent_frame_;
      odom_msg.child_frame_id = child_frame_;

      // Copy pose information into the odometry message.
      odom_msg.pose.pose.position = pose.position;
      odom_msg.pose.pose.orientation = pose.orientation;

       // Initialize pose and twist covariance values to zero.
      for (size_t i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
      }

      // Set pose covariance diagonal values.
      // These values represent confidence/uncertainty in the pose estimate.
      odom_msg.pose.covariance[0]  = cov_pose_;
      odom_msg.pose.covariance[7]  = cov_pose_;
      odom_msg.pose.covariance[14] = cov_pose_;
      odom_msg.pose.covariance[21] = cov_pose_;
      odom_msg.pose.covariance[28] = cov_pose_;
      odom_msg.pose.covariance[35] = cov_pose_;


      // Set twist covariance diagonal values.
      // These values represent uncertainty in velocity/angular velocity.
      odom_msg.twist.covariance[0]  = cov_twist_;
      odom_msg.twist.covariance[7]  = cov_twist_;
      odom_msg.twist.covariance[14] = cov_twist_;
      odom_msg.twist.covariance[21] = cov_twist_;
      odom_msg.twist.covariance[28] = cov_twist_;
      odom_msg.twist.covariance[35] = cov_twist_;

      // Set twist covariance diagonal values.
      // These values represent uncertainty in velocity/angular velocity.
      if (zero_twist_) {
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
      }

       // Publish the odometry message.
      odom_pub_->publish(odom_msg);
    }
  }

  void watchdogCallback()
  {
    // Check how much time has passed since the last PoseArray message.
    auto now = this->now();
    const double dt = (now - last_msg_time_wall_).seconds();

    // Warn if no pose data has been received recently.
    if (dt > 2.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 4000,
        "No PoseArray received recently on %s (%.2f s since last message).",
        input_topic_.c_str(), dt);
    }
  }

private:
 // Topic and frame configuration.
  std::string input_topic_;
  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;
// Index of the pose to select from the PoseArray.
  int pose_index_;
// Publishing options.
  bool publish_odom_;
  bool publish_tf_;
// Covariance values used in the odometry message.
  double cov_pose_;
  double cov_twist_;
 // Option to publish zero velocity when no twist data is available.
  bool zero_twist_;

// ROS 2 subscriber, publisher, TF broadcaster, and timer.
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
 // Last time a PoseArray message was received.
  rclcpp::Time last_msg_time_wall_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GtPoseArrayTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
