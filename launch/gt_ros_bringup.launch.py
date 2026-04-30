from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge /clock
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),

        # Bridge IMU
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
            output="screen",
        ),

        # Bridge dynamic pose -> PoseArray and remap to /gt_pose_array
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"],
            remappings=[("/world/default/dynamic_pose/info", "/gt_pose_array")],
            output="screen",
        ),

        # Your TF + /odom_gt publisher
        Node(
            package="my_cpp20_pubsub",
            executable="gt_posearray_tf",
            name="gt_posearray_tf",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        # Optional: map -> odom static TF (helps RViz)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0","0","0","0","0","0","map","odom"],
            output="screen",
        ),
    ])
