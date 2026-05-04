from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge the simulation clock from Gazebo to ROS 2.
        # This allows the ROS 2 nodes to use the same simulation time as Gazebo.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),

         # Bridge the IMU sensor data from the Gazebo drone model to ROS 2.
        # The IMU provides motion-related data such as orientation, angular velocity,
        # and linear acceleration.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
            output="screen",
        ),

       # Bridge the dynamic pose information from Gazebo to ROS 2 as a PoseArray.
        # This topic contains pose information of models in the simulation.
        # The topic is remapped to /gt_pose_array to make it easier for the custom
        # localization/TF node to read.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"],
            remappings=[("/world/default/dynamic_pose/info", "/gt_pose_array")],
            output="screen",
        ),

      # Start the custom ground-truth pose to TF and odometry publisher.
        # This node reads the Gazebo pose array and publishes the drone pose as:
        # - TF frames for localization
        # - /odom_gt for ground-truth odometry
        Node(
            package="my_cpp20_pubsub",
            executable="gt_posearray_tf",
            name="gt_posearray_tf",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

       # Publish a static transform between the map and odom frames.
        # This helps connect the TF tree for visualization and SLAM/RViz usage.
        # Since all values are zero, map and odom are aligned at the same origin.
       
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0","0","0","0","0","0","map","odom"],
            output="screen",
        ),
    ])
