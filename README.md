# Robotics-AI

This project focuses on building a simulated autonomous drone system capable of detecting gas leaks and reacting intelligently to high-risk areas. The system is developed using PX4 SITL, Gazebo, and ROS 2, and combines perception, mapping, and decision-making into a complete pipeline.

## What This Project Does

The drone operates in a simulated environment where it:

- Receives sensor data using LiDAR and simulated gas values
- Understands its position using ROS 2 and transform frames (TF)
- Builds a map of the environment using SLAM
- Detects high gas concentration areas
- Decides how to react by switching between search, investigate, and hover states

## System Overview

The system is built as a pipeline.

### Simulation Layer

- PX4 SITL as the flight controller
- Gazebo as the simulation environment with sensors

### Middleware Layer: ROS 2

- Topic communication
- Sensor data bridging
- Transform frame handling

### Perception and Mapping

- LiDAR scan processing
- SLAM Toolbox for mapping

### AI Decision Layer

- Rule-based logic using simulated gas concentration values
- Drone behavior states:
  - SEARCH
  - INVESTIGATE
  - CONFIRM_HOVER

### Control Layer

- Offboard control through PX4
- Position setpoints sent from ROS 2
