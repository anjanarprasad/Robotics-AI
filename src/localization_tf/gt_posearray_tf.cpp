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

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (publish_odom_) {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10));
    }

    sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GtPoseArrayTfNode::poseArrayCallback, this, _1));

    watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&GtPoseArrayTfNode::watchdogCallback, this));

    last_msg_time_wall_ = this->now();

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
    last_msg_time_wall_ = this->now();

    if (!msg) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received null PoseArray pointer.");
      return;
    }

    if (msg->poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received PoseArray with zero poses.");
      return;
    }

    if (pose_index_ < 0 || static_cast<size_t>(pose_index_) >= msg->poses.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "pose_index=%d out of range for PoseArray size=%zu",
        pose_index_, msg->poses.size());
      return;
    }

    const auto & pose = msg->poses[pose_index_];

    builtin_interfaces::msg::Time stamp = msg->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = this->get_clock()->now();
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = parent_frame_;
      tf_msg.child_frame_id = child_frame_;
      tf_msg.transform.translation.x = pose.position.x;
      tf_msg.transform.translation.y = pose.position.y;
      tf_msg.transform.translation.z = pose.position.z;
      tf_msg.transform.rotation = pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }

    if (publish_odom_) {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = stamp;
      odom_msg.header.frame_id = parent_frame_;
      odom_msg.child_frame_id = child_frame_;

      odom_msg.pose.pose.position = pose.position;
      odom_msg.pose.pose.orientation = pose.orientation;

      for (size_t i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
      }

      odom_msg.pose.covariance[0]  = cov_pose_;
      odom_msg.pose.covariance[7]  = cov_pose_;
      odom_msg.pose.covariance[14] = cov_pose_;
      odom_msg.pose.covariance[21] = cov_pose_;
      odom_msg.pose.covariance[28] = cov_pose_;
      odom_msg.pose.covariance[35] = cov_pose_;

      odom_msg.twist.covariance[0]  = cov_twist_;
      odom_msg.twist.covariance[7]  = cov_twist_;
      odom_msg.twist.covariance[14] = cov_twist_;
      odom_msg.twist.covariance[21] = cov_twist_;
      odom_msg.twist.covariance[28] = cov_twist_;
      odom_msg.twist.covariance[35] = cov_twist_;

      if (zero_twist_) {
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
      }

      odom_pub_->publish(odom_msg);
    }
  }

  void watchdogCallback()
  {
    auto now = this->now();
    const double dt = (now - last_msg_time_wall_).seconds();

    if (dt > 2.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 4000,
        "No PoseArray received recently on %s (%.2f s since last message).",
        input_topic_.c_str(), dt);
    }
  }

private:
  std::string input_topic_;
  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  int pose_index_;
  bool publish_odom_;
  bool publish_tf_;
  double cov_pose_;
  double cov_twist_;
  bool zero_twist_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
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